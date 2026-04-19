import React from 'react';
import { NextPage, NextPageContext } from 'next';

interface ErrorProps {
  statusCode?: number;
}

const ErrorPage: NextPage<ErrorProps> = ({ statusCode }) => {
  return (
    <div className="cyber-theme h-screen flex flex-col justify-center items-center">
      <p className="text-sm font-medium text-red-400 uppercase tracking-widest">
        {statusCode ?? 'Error'}
      </p>
      <h1 className="mt-2 text-4xl font-bold text-white">Oops!</h1>
      <p className="mt-2 text-white/50 text-sm">
        {statusCode
          ? `An error ${statusCode} occurred on the server.`
          : 'An error occurred on the client.'}
      </p>
    </div>
  );
};

// Static method to retrieve the error status code
ErrorPage.getInitialProps = async ({ res, err }: NextPageContext): Promise<ErrorProps> => {
  const statusCode = res ? res.statusCode : err ? err.statusCode : 404;
  return { statusCode };
};

export default ErrorPage;
