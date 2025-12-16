# Constants for the RAG Chatbot application

# Qdrant settings
QDRANT_HOST = "localhost"
QDRANT_PORT = 6333
QDRANT_COLLECTION_NAME = "textbook_content"

# Similarity settings
SIMILARITY_THRESHOLD = 0.5

# Confidence score ranges
MIN_CONFIDENCE_SCORE = 0.0
MAX_CONFIDENCE_SCORE = 1.0

# Error messages
ERROR_QUERY_OUTSIDE_SCOPE = "The requested information is not available in the textbook. I can only answer questions based on the content provided in the Physical AI & Humanoid Robotics textbook."

# HTTP status codes
HTTP_422_UNPROCESSABLE_ENTITY = 422