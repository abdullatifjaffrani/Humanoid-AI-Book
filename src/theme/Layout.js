import React, { useState } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ChatWidget from '../components/rag-chatbot/ChatWidget';
import TextSelectionHandler from '../components/rag-chatbot/TextSelectionHandler';

const Layout = ({ children }) => {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const [selectedText, setSelectedText] = useState(null);

  // Make the setSelectedText function available globally for the text selection handler
  window.ChatWidgetRef = { current: { askAboutSelection: setSelectedText } };

  return (
    <TextSelectionHandler>
      <div className="main-wrapper">
        {children}
        <ChatWidget selectedText={selectedText} />
      </div>
    </TextSelectionHandler>
  );
};

export default Layout;