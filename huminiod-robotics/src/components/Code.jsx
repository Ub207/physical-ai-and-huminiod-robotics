import React from 'react';

// A simple Code component for displaying code examples
const Code = ({ children, language = '', ...props }) => {
  return (
    <div className={`code-block-${language}`} {...props}>
      <pre>
        <code>
          {children}
        </code>
      </pre>
    </div>
  );
};

export default Code;