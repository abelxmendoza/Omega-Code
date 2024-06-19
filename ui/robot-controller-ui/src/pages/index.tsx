import Head from 'next/head';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import ControlPanel from '../components/ControlPanel';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';

const Home: React.FC = () => {
  return (
    <div className="min-h-screen bg-gray-50">
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <Header />
      <main className="p-4 space-y-4">
        <div className="flex justify-center items-center space-x-4">
          <ControlPanel />
          <VideoFeed />
          <ControlPanel />
        </div>
        <div className="flex flex-col items-center space-y-4 mt-4">
          <SpeedControl />
          <CommandLog />
        </div>
      </main>
    </div>
  );
};

export default Home;
