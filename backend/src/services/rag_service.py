import time
from typing import List, Dict, Optional, Any
from uuid import UUID
from ..models.query import QueryRequest, QueryModel
from ..models.response import ResponseModel
from ..models.chunk import ChunkModel
from ..config.settings import settings
from .retrieval import RetrievalService
from .validation import ValidationService
from .citation import CitationService
from ..utils.performance_monitor import performance_monitor
import logging
from openai import OpenAI
from ..config.constants import MIN_CONFIDENCE_SCORE, MAX_CONFIDENCE_SCORE


class RAGService:
    """
    Main RAG (Retrieval-Augmented Generation) service
    Orchestrates the process of retrieving relevant text chunks and generating grounded responses
    """

    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.validation_service = ValidationService()
        self.citation_service = CitationService()
        # Initialize OpenAI client with Google's OpenAI-compatible endpoint
        self.client = OpenAI(
            api_key=settings.GEMINI_API_KEY,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

    @performance_monitor.measure_time
    def process_query(self, query_request: QueryRequest) -> Optional[ResponseModel]:
        """
        Process a user query and return a grounded response with citations
        """
        start_time = time.time()

        try:
            # Check if user has provided specific text to use for context
            if query_request.selected_text:
                # When user provides selected text, we should retrieve additional relevant information
                # from the textbook to provide detailed explanations about the selection
                selected_chunk = ChunkModel(
                    document_id="selected_text",
                    content=query_request.selected_text,
                    metadata={
                        'source_reference': 'User-selected text',
                        'page_number': None,
                        'chapter': None,
                        'section': None
                    }
                )

                # If the query is about the selected text (e.g., "what is my selection?" or similar),
                # retrieve additional relevant chunks about the selected text to provide more detail
                try:
                    if self._is_query_about_selection(query_request.query, query_request.selected_text):
                        # Retrieve additional relevant chunks based on the selected text content
                        additional_chunks = self.retrieval_service.retrieve_chunks(query_request.selected_text)
                        # Combine selected text with additional relevant information
                        relevant_chunks = [selected_chunk] + additional_chunks if additional_chunks else [selected_chunk]
                    else:
                        # For other queries with selected text, use both the query and selected text for retrieval
                        combined_query = f"{query_request.query} {query_request.selected_text}"
                        additional_chunks = self.retrieval_service.retrieve_chunks(combined_query)
                        # Combine selected text with query-relevant chunks
                        relevant_chunks = [selected_chunk] + additional_chunks if additional_chunks else [selected_chunk]
                except Exception as e:
                    logging.error(f"Vector store unavailable when processing selected text query: {query_request.query}. Error: {str(e)}")
                    # Return a response indicating the system is temporarily unavailable
                    return ResponseModel(
                        content="The system is temporarily unable to retrieve information from the textbook. Please try again later.",
                        citations=[],
                        confidence_score=0.0
                    )

                chunk_count = len(relevant_chunks)
            else:
                # Step 1: Retrieve relevant text chunks based on the query
                try:
                    relevant_chunks = self.retrieval_service.retrieve_chunks(query_request.query)
                except Exception as e:
                    logging.error(f"Vector store unavailable when processing query: {query_request.query}. Error: {str(e)}")
                    # Return a response indicating the system is temporarily unavailable
                    return ResponseModel(
                        content="The system is temporarily unable to retrieve information from the textbook. Please try again later.",
                        citations=[],
                        confidence_score=0.0
                    )

                chunk_count = len(relevant_chunks)

                # Step 2: If no relevant chunks found, return None to indicate the query cannot be answered
                if not relevant_chunks:
                    logging.info(f"No relevant chunks found for query: {query_request.query}")
                    return None

            # Step 3: Generate a response based on the context (either selected text or retrieved chunks)
            response_content = self._generate_response(query_request.query, relevant_chunks)

            # Step 4: Validate that the response is grounded in the provided context
            is_valid = self.validation_service.validate_response(response_content, relevant_chunks)

            if not is_valid:
                logging.warning(f"Response failed validation: {response_content}")
                # Return a response indicating the issue
                return ResponseModel(
                    content="The response could not be properly validated against the source material.",
                    citations=[],
                    confidence_score=0.0
                )

            # Step 5: Generate citations for the response
            citations = self.citation_service.generate_citations(relevant_chunks, response_content)

            # Step 6: Calculate confidence score based on relevance of provided context
            confidence_score = self._calculate_confidence_score(relevant_chunks)

            # Step 7: Create and return the response model
            response_model = ResponseModel(
                content=response_content,
                citations=citations,
                confidence_score=confidence_score
            )

            # Log performance metrics
            response_time = time.time() - start_time
            performance_monitor.log_query_performance(
                query=query_request.query,
                response_time=response_time,
                retrieved_chunks=chunk_count
            )

            return response_model

        except Exception as e:
            response_time = time.time() - start_time
            logging.error(f"Error processing query after {response_time:.3f}s: {str(e)}")
            raise

    def _is_query_about_selection(self, query: str, selected_text: str) -> bool:
        """
        Check if the query is asking about the selected text itself
        """
        query_lower = query.lower().strip()
        selected_lower = selected_text.lower().strip()

        # Check for common phrases that indicate the user is asking about their selection
        about_selection_phrases = [
            "what is my selection",
            "what is this selection",
            "what is selected",
            "tell me about this",
            "tell me about the selection",
            "explain this",
            "explain the selection",
            "what does this mean",
            "what does this refer to",
            "what is this"
        ]

        for phrase in about_selection_phrases:
            if phrase in query_lower:
                return True

        # If the query is very short and similar to the selected text, it's likely about the selection
        if len(query_lower.split()) <= 3 and selected_lower in query_lower:
            return True

        return False

    def _generate_response(self, query: str, chunks: List[ChunkModel]) -> str:
        """
        Generate a response using OpenAI client with Google's OpenAI-compatible endpoint
        based on the query and retrieved chunks. Falls back to simple extraction if API quota is exceeded.
        """
        try:
            # Format the context from retrieved chunks
            context = "\n\n".join([chunk.content for chunk in chunks])

            # Create the system message with instructions
            system_message = "You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context. If the answer is not in the context, clearly state that the information is not available in the textbook."

            # Create the user message with context and query
            user_message = f"""
            Based on the following context from the Physical AI & Humanoid Robotics textbook, answer the question.
            If the answer is not available in the context, clearly state that the information is not available in the textbook.

            Context: {context}

            Question: {query}

            Answer:
            """

            # Use OpenAI client with Google's OpenAI-compatible endpoint
            response = self.client.chat.completions.create(
                model=settings.GEMINI_MODEL,  # Using the configured model name
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                max_tokens=500,
                temperature=0.3,
            )

            # Extract the generated text
            generated_text = response.choices[0].message.content.strip()

            return generated_text
        except Exception as e:
            logging.error(f"Error generating response: {str(e)}")

            # Check if it's a quota error or any other API-related error and provide fallback
            error_str = str(e).lower()
            if ("quota" in error_str or
                "429" in str(e) or
                "resource_exhausted" in error_str or
                "rate limit" in error_str or
                "exceeded" in error_str):
                logging.info("Using fallback response generation due to API limits")
                try:
                    return self._generate_fallback_response(query, chunks)
                except Exception as fallback_error:
                    logging.error(f"Fallback response generation also failed: {str(fallback_error)}")
                    return "I'm sorry, I'm currently experiencing high demand and cannot generate a response. Please try again later."
            else:
                # For other errors, log and return a meaningful message instead of raising
                logging.error(f"Non-quota related error in response generation: {str(e)}")
                return "I'm sorry, I encountered an error while processing your request. Please try again."

    def _generate_fallback_response(self, query: str, chunks: List[ChunkModel]) -> str:
        """
        Fallback response generation that extracts information from context without using API
        """
        try:
            # Simple approach: look for relevant sentences in the context that might answer the query
            context_text = "\n\n".join([chunk.content for chunk in chunks])

            # If the query is a simple greeting, return a greeting
            if query.lower().strip() in ["hi", "hello", "hey", "greetings", "help"]:
                return "Hello! I'm your Physical AI & Humanoid Robotics assistant. I can help answer questions about the textbook content. Please ask a question related to the textbook."

            # For other queries, return the relevant context text
            # This is a basic fallback - in a real implementation, you'd want more sophisticated text extraction
            if len(context_text.strip()) > 0:
                # Return the first relevant part of the context
                # In a more sophisticated version, you'd use keyword matching or NLP techniques
                sentences = context_text.split('. ')
                relevant_sentences = []

                query_lower = query.lower()
                for sentence in sentences:
                    if any(keyword in sentence.lower() for keyword in query_lower.split()[:3]):  # Check first 3 words of query
                        relevant_sentences.append(sentence.strip())

                if relevant_sentences:
                    return ". ".join(relevant_sentences[:3]) + "."  # Return up to 3 relevant sentences
                else:
                    # If no specific match, return the first part of the context
                    return context_text[:500] + "..." if len(context_text) > 500 else context_text
            else:
                return "No relevant information found in the textbook to answer your question."

        except Exception as e:
            logging.error(f"Error in fallback response generation: {str(e)}")
            return "I'm sorry, I couldn't generate a response at this time due to technical constraints. Please try again later."

    def _calculate_confidence_score(self, chunks: List[ChunkModel]) -> float:
        """
        Calculate a confidence score based on the relevance of retrieved chunks
        """
        if not chunks:
            return MIN_CONFIDENCE_SCORE

        # Calculate average similarity score of the chunks
        similarity_scores = []
        for chunk in chunks:
            if chunk.metadata and 'similarity_score' in chunk.metadata:
                similarity_scores.append(chunk.metadata['similarity_score'])
            else:
                # Default to 0.5 if no similarity score available
                similarity_scores.append(0.5)

        if similarity_scores:
            avg_score = sum(similarity_scores) / len(similarity_scores)
        else:
            avg_score = 0.5

        # Ensure the score is within the valid range
        return max(MIN_CONFIDENCE_SCORE, min(MAX_CONFIDENCE_SCORE, avg_score))


# Global instance of RAG service
rag_service = RAGService()