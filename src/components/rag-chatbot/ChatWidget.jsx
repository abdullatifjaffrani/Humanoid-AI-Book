import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';
import config from './config';

const ChatWidget = ({ selectedText = null }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { text: config.INITIAL_MESSAGE, sender: 'bot', timestamp: new Date() }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle selected text by opening the chat and asking about it
  useEffect(() => {
    if (selectedText) {
      // Open the chat when selected text is provided
      setIsOpen(true);

      // Auto-ask about the selected text
      const autoQuery = `Tell me about this selection: "${selectedText}"`;
      setInput(autoQuery);

      // Simulate pressing enter to send the query
      const autoSendMessage = async () => {
        if (autoQuery.trim() && !isLoading) {
          const userMessage = { text: autoQuery, sender: 'user', timestamp: new Date() };
          setMessages(prev => [...prev, userMessage]);

          try {
            const response = await fetch(`${config.API_BASE_URL}${config.CHAT_ENDPOINT}`, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
                'Accept': 'application/json'
              },
              body: JSON.stringify({
                query: autoQuery,
                selected_text: selectedText,
                session_id: null,
                metadata: {}
              })
            });

            if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();

            const botMessage = {
              text: data.answer || 'Sorry, I could not find an answer to that question in the textbook.',
              sender: 'bot',
              citations: data.citations || [],
              confidence: data.confidence_score || 0,
              timestamp: new Date()
            };

            setMessages(prev => [...prev, botMessage]);
          } catch (error) {
            console.error('Error sending auto-message:', error);
            let errorMessageText = 'Sorry, there was an error processing your request.';

            if (error.name === 'TypeError' && error.message.includes('fetch')) {
              errorMessageText = 'Unable to connect to the AI service. Please make sure the backend server is running on http://localhost:8000.';
            } else if (error.message.includes('429')) {
              errorMessageText = 'Rate limit exceeded. Please wait a moment before asking another question.';
            } else if (error.message.includes('404')) {
              errorMessageText = 'The AI service endpoint was not found. Please check if the backend is properly configured.';
            } else if (error.message.includes('NetworkError')) {
              errorMessageText = 'Network error. Please check your connection and make sure the backend server is running.';
            }

            const errorMessage = {
              text: errorMessageText,
              sender: 'bot',
              timestamp: new Date()
            };
            setMessages(prev => [...prev, errorMessage]);
          }
        }
      };

      // Delay the auto-send slightly to allow UI to update
      const timer = setTimeout(autoSendMessage, 300);
      return () => clearTimeout(timer);
    }
  }, [selectedText]);

  const toggleChat = () => setIsOpen(!isOpen);

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage = { text: input, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Send request to your backend RAG service
      const response = await fetch(`${config.API_BASE_URL}${config.CHAT_ENDPOINT}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        },
        body: JSON.stringify({
          query: input,
          selected_text: selectedText,
          session_id: null,
          metadata: {}
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        text: data.answer || 'Sorry, I could not find an answer to that question in the textbook.',
        sender: 'bot',
        citations: data.citations || [],
        confidence: data.confidence_score || 0,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      let errorMessageText = 'Sorry, there was an error processing your request.';

      // Provide more specific error messages
      if (error.name === 'TypeError' && error.message.includes('fetch')) {
        errorMessageText = 'Unable to connect to the AI service. Please make sure the backend server is running on http://localhost:8000.';
      } else if (error.message.includes('429')) {
        errorMessageText = 'Rate limit exceeded. Please wait a moment before asking another question.';
      } else if (error.message.includes('404')) {
        errorMessageText = 'The AI service endpoint was not found. Please check if the backend is properly configured.';
      } else if (error.message.includes('NetworkError')) {
        errorMessageText = 'Network error. Please check your connection and make sure the backend server is running.';
      }

      const errorMessage = {
        text: errorMessageText,
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chat-widget">
      {!isOpen ? (
        <button className="chat-toggle" onClick={toggleChat} aria-label="Open AI Assistant">
          <span className="chat-icon">💬</span>
          <span className="chat-text">AI Assistant</span>
        </button>
      ) : (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-header-content">
              <span className="chat-header-title">Physical AI Assistant</span>
              <button onClick={toggleChat} className="close-btn" aria-label="Close chat">
                ✕
              </button>
            </div>
          </div>
          <div className="chat-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`message ${msg.sender}`}>
                <div className="message-text">{msg.text}</div>
                {msg.citations && msg.citations.length > 0 && (
                  <div className="citations">
                    <details className="citations-details">
                      <summary>Sources</summary>
                      <ul>
                        {msg.citations.map((cit, i) => (
                          <li key={i}>
                            <span className="citation-source">{cit.source_reference}</span>
                            {cit.page_number && <span className="citation-page"> (p. {cit.page_number})</span>}
                          </li>
                        ))}
                      </ul>
                    </details>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className="message bot">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={sendMessage} className="chat-input-form">
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask about Physical AI & Humanoid Robotics..."
              disabled={isLoading}
              aria-label="Type your question"
            />
            <button type="submit" disabled={isLoading} aria-label="Send message">
              {isLoading ? '...' : '→'}
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;