@echo off
echo Starting RAG Chatbot Services...

echo Starting RAG Server on port 5007...
start "RAG Server" cmd /k "cd /d %~dp0 && node server-upgradeable.mjs"

timeout /t 5 /nobreak > nul

echo Starting MCP Adapter on port 5002...
start "MCP Adapter" cmd /k "cd /d %~dp0 && node mcp-adapter.js"

echo Services started! Check the console windows for any errors.
echo The RAG server should be available at http://localhost:5007
echo The MCP adapter should be available at http://localhost:5002
echo The chatbot widget should now be able to connect to the MCP adapter at http://localhost:5002