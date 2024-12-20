import React from 'react';

interface ErrorProps {
  statusCode?: number;
}

const ErrorPage: React.FC<ErrorProps> = ({ statusCode }) => {
  return (
    <div className="h-screen flex flex-col justify-center items-center bg-gray-100">
      <h1 className="text-4xl font-bold">Oops!</h1>
      <p className="mt-2">
        {statusCode
          ? `An error ${statusCode} occurred on the server.`
          : 'An error occurred on the client.'}
      </p>
    </div>
  );
};

ErrorPage.getInitialProps = ({ res, err }: any) => {
  const statusCode = res ? res.statusCode : err ? err.statusCode : 404;
  return { statusCode };
};

export default ErrorPage;
