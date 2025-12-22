import axios from 'axios';

async function testConnections() {
  console.log('Testing RAG Chatbot Service Connections...\n');

  // Test RAG Server
  try {
    console.log('1. Testing RAG Server at http://localhost:5007/health...');
    const ragResponse = await axios.get('http://localhost:5007/health');
    console.log('   ✓ RAG Server is running:', ragResponse.data.status);
    console.log('   ✓ Document count:', ragResponse.data.documentCount);
  } catch (error) {
    console.log('   ✗ RAG Server connection failed:', error.message);
  }

  // Test MCP Adapter
  try {
    console.log('\n2. Testing MCP Adapter at http://localhost:5002/health...');
    const mcpResponse = await axios.get('http://localhost:5002/health');
    console.log('   ✓ MCP Adapter is running:', mcpResponse.data.status);
    console.log('   ✓ Connected to RAG server:', mcpResponse.data.rag_server_url);
  } catch (error) {
    console.log('   ✗ MCP Adapter connection failed:', error.message);
  }

  // Test MCP Tools endpoint
  try {
    console.log('\n3. Testing MCP Tools endpoint...');
    const toolsResponse = await axios.get('http://localhost:5002/tools');
    console.log('   ✓ MCP Tools endpoint available');
    console.log('   ✓ Available tools:', toolsResponse.data.tools.map(t => t.name).join(', '));
  } catch (error) {
    console.log('   ✗ MCP Tools endpoint failed:', error.message);
  }

  // Test a simple search if services are available
  try {
    console.log('\n4. Testing search functionality...');
    const searchResponse = await axios.get('http://localhost:5007/api/search?q=test&limit=1');
    console.log('   ✓ Search API is working');
    console.log('   ✓ Found', searchResponse.data.count, 'results');
  } catch (error) {
    console.log('   ✗ Search API test failed:', error.message);
  }

  console.log('\nTest completed!');
}

// Run the test
testConnections().catch(console.error);