import React from 'react';
import Content from '@theme-original/DocItem/Content';
import TranslationButton from '@site/src/components/translation/TranslationButton';

export default function ContentWrapper(props) {
  return (
    <>
      {/* Translation button at the start of every chapter */}
      <TranslationButton />
      <Content {...props} />
    </>
  );
}
