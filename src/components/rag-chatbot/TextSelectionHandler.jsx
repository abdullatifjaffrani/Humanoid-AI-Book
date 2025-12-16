import React, { useState, useEffect } from 'react';

const TextSelectionHandler = ({ children }) => {
  const [selectedText, setSelectedText] = useState(null);
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      if (text.length > 0) {
        // Get the bounding rectangle of the selection
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        // Position the button near the selection
        setButtonPosition({
          x: rect.left + window.scrollX,
          y: rect.top + window.scrollY - 40  // Position above the selection
        });
        
        setSelectedText(text);
        setShowButton(true);
      } else {
        setShowButton(false);
      }
    };

    const handleClick = () => {
      // Hide the button when user clicks anywhere
      setTimeout(() => {
        if (!window.getSelection().toString().trim()) {
          setShowButton(false);
        }
      }, 100);
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('click', handleClick);
    
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('click', handleClick);
    };
  }, []);

  const handleAskQuestion = () => {
    // This will be handled by the parent component or through context
    // For now, we'll just pass the selected text to a callback
    if (window.ChatWidgetRef && window.ChatWidgetRef.current) {
      window.ChatWidgetRef.current.askAboutSelection(selectedText);
    }
    setShowButton(false);
  };

  return (
    <div style={{ position: 'relative' }}>
      {children}
      {showButton && (
        <button
          className="text-selection-ask-button"
          style={{
            position: 'fixed',
            left: buttonPosition.x,
            top: buttonPosition.y,
            zIndex: 10000,
            background: '#4CAF50',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            padding: '6px 12px',
            fontSize: '14px',
            cursor: 'pointer',
            boxShadow: '0 2px 6px rgba(0,0,0,0.2)',
          }}
          onClick={handleAskQuestion}
        >
          ?
        </button>
      )}
    </div>
  );
};

export default TextSelectionHandler;