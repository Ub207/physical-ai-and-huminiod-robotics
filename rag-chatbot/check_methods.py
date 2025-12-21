from qdrant_client import QdrantClient
from config.settings import settings

# Initialize client
qdrant_client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
)

# Check available methods
methods = [method for method in dir(qdrant_client) if not method.startswith('_')]
print("Available methods in QdrantClient:")
for method in sorted(methods):
    print(f"  - {method}")

# Check if search method exists
print(f"\nHas 'search' method: {hasattr(qdrant_client, 'search')}")
print(f"Has 'search_points' method: {hasattr(qdrant_client, 'search_points')}")
print(f"Has 'grpc_points_client' method: {hasattr(qdrant_client, 'grpc_points_client')}")

# Let's also check the http-based client methods if available
if hasattr(qdrant_client, '_client'):
    print(f"\nHas '_client': {hasattr(qdrant_client, '_client')}")
    http_methods = [method for method in dir(qdrant_client._client) if not method.startswith('_')]
    print("Available methods in internal client (first 20):")
    for method in sorted(http_methods)[:20]:
        print(f"  - {method}")