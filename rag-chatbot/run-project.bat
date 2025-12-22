@echo off
echo ================================================
echo Starting Physical AI Humanoid Robotics Platform
echo ================================================

echo.
echo Starting RAG Server on port 5007...
start "RAG Server - Port 5007" cmd /k "cd /d %~dp0 && echo RAG Server starting... && echo. && node server-upgradeable.mjs"

echo.
echo Waiting for RAG server to initialize...
timeout /t 10 /nobreak > nul

echo.
echo Starting MCP Adapter on port 5002...
echo Note: MCP Adapter will attempt to connect to RAG server at http://localhost:5007
start "MCP Adapter - Port 5002" cmd /k "cd /d %~dp0 && set RAG_SERVER_URL=http://localhost:5007 && set RAG_MCP_PORT=5002 && echo Environment variables set && node mcp-adapter.js"

echo.
echo ================================================
echo Services started successfully!
echo ================================================
echo RAG Server: http://localhost:5007
echo MCP Adapter: http://localhost:5002 (if environment variables are properly set)
echo.
echo The chatbot widget is configured to connect to http://localhost:5002
echo.
echo If MCP is still on port 5001, please manually set the environment variables:
echo   set RAG_SERVER_URL=http://localhost:5007
echo   set RAG_MCP_PORT=5002
echo   node mcp-adapter.js
echo ================================================