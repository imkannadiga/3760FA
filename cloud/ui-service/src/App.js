import logo from './logo.svg';
import './App.css';
import MultiMapViewer from './componants/MultiMapViewer';

function App() {
  return (
    <div className="App">
      <h1>Costmap viewer</h1>
      <MultiMapViewer apiUrl="http://localhost:10002/api/costmap/download_map" />
    </div>
  );
}

export default App;
