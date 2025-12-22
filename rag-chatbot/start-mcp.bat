@echo off
echo Starting MCP Adapter with correct configuration...

REM Set environment variables and start the MCP adapter
set RAG_SERVER_URL=http://localhost:5007
set RAG_MCP_PORT=5002
echo Environment variables set:
echo RAG_SERVER_URL=%RAG_SERVER_URL%
echo RAG_MCP_PORT=%RAG_MCP_PORT%

echo.
echo Starting MCP Adapter...
node mcp-adapter.js

pause