from typing import List
from ..models.chunk import ChunkModel
import logging
import re


class ValidationService:
    """
    Service for validating that generated responses are grounded in the retrieved content
    and preventing hallucination
    """

    def __init__(self):
        pass

    def validate_response(self, response: str, chunks: List[ChunkModel]) -> bool:
        """
        Validate that the response is grounded in the provided chunks
        Returns True if the response is valid, False otherwise
        """
        try:
            # Check if the response contains information that can be traced back to the chunks
            is_content_valid = self._validate_content_grounding(response, chunks)

            # Check if the response contains hallucinated information
            is_not_hallucinated = self._check_for_hallucination(response, chunks)

            # Check if the response properly acknowledges when information is not available
            is_acknowledged_outside_scope = self._check_outside_scope_acknowledgement(response, chunks)

            return is_content_valid and is_not_hallucinated and is_acknowledged_outside_scope

        except Exception as e:
            logging.error(f"Error validating response: {str(e)}")
            return False

    def _validate_content_grounding(self, response: str, chunks: List[ChunkModel]) -> bool:
        """
        Check if the response content is grounded in the provided chunks
        """
        if not chunks:
            return True  # If there are no chunks, any response that acknowledges this is valid

        # Check if any chunk has 'User-selected text' as source reference
        has_user_selected_text = any(
            chunk.metadata and chunk.metadata.get('source_reference') == 'User-selected text'
            for chunk in chunks
        )

        # Convert response and chunks to lowercase for comparison
        response_lower = response.lower()

        # Check if the response acknowledges that information is not available in the textbook
        # If so, this is valid when there are chunks but they don't contain the requested information
        acknowledgment_phrases = [
            "not available in the textbook",
            "not found in the textbook",
            "information not available",
            "no information provided",
            "not mentioned in the textbook",
            "i cannot find this information in the textbook",
            "no relevant information found",
            "not in the textbook"
        ]

        if any(phrase in response_lower for phrase in acknowledgment_phrases):
            return True  # Acknowledging absence of information is valid

        # Extract key phrases or concepts from the response
        # This is a simplified approach - in a real implementation, we'd use more sophisticated NLP
        response_words = set(re.findall(r'\b\w+\b', response_lower))

        # Check if there's significant overlap between response and chunks
        chunk_content = " ".join([chunk.content.lower() for chunk in chunks])
        chunk_words = set(re.findall(r'\b\w+\b', chunk_content))

        # For user-selected text, be more permissive since user might ask about the selection itself
        if has_user_selected_text:
            # Check if response contains key content from the selected text
            # This is more lenient for user selection context
            common_words = response_words.intersection(chunk_words)

            # If user is asking about their selection, check if response references the selected content
            if len(common_words) > 0 or len(response_words) == 0:
                return True  # For user-selected context, some overlap is sufficient
            else:
                # Check if response is just acknowledging the selection without adding info
                if "your selection" in response_lower or "selected text" in response_lower:
                    return True

        # Calculate overlap for regular (non-user-selected) content
        common_words = response_words.intersection(chunk_words)
        if len(common_words) == 0:
            return False  # No overlap suggests potential hallucination

        # If at least 30% of unique words in the response appear in the chunks, consider it grounded
        if len(response_words) > 0:
            overlap_ratio = len(common_words) / len(response_words)
            return overlap_ratio >= 0.3
        else:
            return False

    def _check_for_hallucination(self, response: str, chunks: List[ChunkModel]) -> bool:
        """
        Check if the response contains hallucinated information
        """
        # Check if the response claims to have information that isn't in the chunks
        # For now, we'll check if the response says something is in the textbook when it's not

        # If response acknowledges information isn't available in the textbook, it's valid
        # This is valid regardless of whether there are chunks or not
        acknowledgment_phrases = [
            "not available in the textbook",
            "not found in the textbook",
            "information not available",
            "no information provided",
            "not mentioned in the textbook",
            "i cannot find this information in the textbook",
            "no relevant information found",
            "not in the textbook",
            "does not define",
            "does not contain",
            "not specified in the provided text",
            "textbook does not mention",
            "not covered in the material"
        ]

        if any(phrase in response.lower() for phrase in acknowledgment_phrases):
            return True  # Acknowledging absence of information is not hallucination

        # If there are no chunks but the response provides detailed information, it's likely hallucinated
        if len(chunks) == 0:
            # Check if response contains phrases that indicate no information is available
            # If it doesn't contain such phrases but also has no chunks to support it, it's hallucinated
            non_committal_phrases = [
                "not available", "not found", "no information",
                "cannot find", "don't know", "unknown", "not specified",
                "not mentioned", "not provided", "not in the text"
            ]
            has_non_committal = any(phrase in response.lower() for phrase in non_committal_phrases)

            # If it doesn't have non-committal phrases but has no chunks to support it, it's hallucinated
            return has_non_committal

        # Perform a more thorough check for consistency between response and chunks
        # This is a simplified approach - in practice, you'd want more sophisticated NLP
        response_lower = response.lower()
        combined_chunk_content = " ".join([chunk.content.lower() for chunk in chunks])

        # Extract key concepts from the response that should be supported by chunks
        # Look for specific claims or facts that should be in the chunks
        response_sentences = re.split(r'[.!?]+', response)

        # For each sentence, check if it has support in the chunks
        for sentence in response_sentences:
            sentence = sentence.strip().lower()
            if len(sentence) > 10:  # Only check meaningful sentences
                # Skip if it's an acknowledgment sentence
                if any(phrase in sentence for phrase in acknowledgment_phrases):
                    continue

                # Check if sentence content appears in the chunks
                sentence_in_chunks = any(sentence_part in combined_chunk_content
                                       for sentence_part in [sentence[:50], sentence[:30], sentence.split()[:5]]
                                       if len(sentence_part) > 5)

                # If the sentence is not found in chunks and is making a claim, it might be hallucinated
                if not sentence_in_chunks:
                    # Additional check: see if there are keywords that appear in both response and chunks
                    response_words = set(re.findall(r'\b\w+\b', sentence))
                    chunk_words = set(re.findall(r'\b\w+\b', combined_chunk_content))
                    common_words = response_words.intersection(chunk_words)

                    # If there's very little overlap, it might be hallucinated
                    if len(common_words) / len(response_words) < 0.2 if response_words else 0:
                        return False  # Likely hallucinated

        return True

    def _check_outside_scope_acknowledgement(self, response: str, chunks: List[ChunkModel]) -> bool:
        """
        Check if the response properly acknowledges when information is outside the scope of available chunks
        """
        # If there are no chunks (no relevant content found), the response should acknowledge this
        if len(chunks) == 0:
            response_lower = response.lower()
            acknowledgment_phrases = [
                "not available in the textbook",
                "not found in the textbook",
                "information not available",
                "no information provided",
                "not mentioned in the textbook",
                "i cannot find this information in the textbook"
            ]

            return any(phrase in response_lower for phrase in acknowledgment_phrases)

        # If there are chunks, the response should not claim information is unavailable
        # unless it specifically addresses that aspect
        return True


# Global instance of validation service
validation_service = ValidationService()