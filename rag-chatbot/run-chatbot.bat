@echo off
echo Starting RAG Chatbot Services...

echo.
echo 1. Starting RAG Server on port 5007...
start "RAG Server - Port 5007" cmd /k "cd /d %~dp0 && echo Starting RAG Server... && node server-upgradeable.mjs"

echo Waiting for RAG server to start...
timeout /t 10 /nobreak > nul

echo.
echo 2. Starting MCP Adapter on port 5002...
start "MCP Adapter - Port 5002" cmd /k "cd /d %~dp0 && echo Starting MCP Adapter... && node mcp-adapter.js"

echo.
echo Services started successfully!
echo.
echo RAG Server: http://localhost:5007
echo MCP Adapter: http://localhost:5002
echo.
echo The chatbot widget is now configured to connect to http://localhost:5002
echo.
echo To test the services, you can visit:
echo - Health check: http://localhost:5007/health
echo - MCP tools: http://localhost:5002/tools
echo.
echo To index documents, run: curl -X POST http://localhost:5007/api/rebuild-index
echo.
pause