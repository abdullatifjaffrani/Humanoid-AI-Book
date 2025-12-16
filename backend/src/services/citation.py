from typing import List, Dict, Any
from ..models.chunk import ChunkModel
import logging


class CitationService:
    """
    Service for generating proper citations from retrieved text chunks
    """

    def __init__(self):
        pass

    def generate_citations(self, chunks: List[ChunkModel], response: str = None) -> List[Dict[str, Any]]:
        """
        Generate citations for the provided chunks
        """
        try:
            citations = []
            for chunk in chunks:
                citation = self._create_citation(chunk)
                # Only add citations that have meaningful content
                if citation.get('text_excerpt', '').strip():
                    citations.append(citation)

            return citations

        except Exception as e:
            logging.error(f"Error generating citations: {str(e)}")
            raise

    def _create_citation(self, chunk: ChunkModel) -> Dict[str, Any]:
        """
        Create a citation for a single chunk with detailed source tracking as required by constitution
        """
        citation = {
            "text_excerpt": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,  # First 200 chars or full text
            "source_reference": chunk.metadata.get('source_reference', '') if chunk.metadata else '',
            "relevance_score": chunk.metadata.get('similarity_score', 0.0) if chunk.metadata else 0.0,
            "document_id": str(chunk.document_id) if chunk.document_id else '',
            "chunk_id": str(chunk.chunk_id) if chunk.chunk_id else ''  # Include chunk ID for precise tracking as string
        }

        # Add additional metadata if available to ensure proper source attribution
        if chunk.metadata:
            if 'page_number' in chunk.metadata:
                citation['page_number'] = chunk.metadata['page_number']
            if 'chapter' in chunk.metadata:
                citation['chapter'] = chunk.metadata['chapter']
            if 'section' in chunk.metadata:
                citation['section'] = chunk.metadata['section']
            if 'author' in chunk.metadata:
                citation['author'] = chunk.metadata['author']
            if 'title' in chunk.metadata:
                citation['title'] = chunk.metadata['title']
            if 'publication_year' in chunk.metadata:
                citation['publication_year'] = chunk.metadata['publication_year']
            # Include the original source location for verification
            if 'source_location' in chunk.metadata:
                citation['source_location'] = chunk.metadata['source_location']

        # Ensure we have proper source attribution by checking if source_reference exists
        if not citation.get('source_reference') or citation['source_reference'] == '':
            logging.warning(f"Chunk {chunk.chunk_id} has no source_reference - this violates constitution requirements for proper citation")
            citation['source_reference'] = 'Source reference not available'

        return citation

    def format_citation(self, citation: Dict[str, Any]) -> str:
        """
        Format a citation in a human-readable format
        """
        source_ref = citation.get('source_reference', 'Unknown source')
        page_info = f", page {citation.get('page_number')}" if citation.get('page_number') else ""
        chapter_info = f", Chapter {citation.get('chapter')}" if citation.get('chapter') else ""
        section_info = f", Section {citation.get('section')}" if citation.get('section') else ""

        formatted = f"{source_ref}{page_info}{chapter_info}{section_info}"
        return formatted


# Global instance of citation service
citation_service = CitationService()