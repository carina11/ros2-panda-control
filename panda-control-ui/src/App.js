import logo from './logo.svg';
import './App.css';
import { Joystick } from 'react-joystick-component';
import { useRef, useState } from 'react';

// function handleMove(e){
//   console.log(e)
// }

// function handleStop(e) {
//   console.log(e)
// }

function App() {
  const [messages, setMessages] = useState([]);
  const ws = useRef(null);

  ws.current = new WebSocket('ws://localhost:8080');
  ws.current.onopen = () => {
    console.log("WebSocket connected");
    ws.current.send("Hello from react");
  }

  ws.current.onmessage = (event) => {
    console.log("Received:", event.data);
  }

  ws.current.onclose = () => {
    console.log("WebSocket disconnected");
  }

  ws.current.onerror = (error) => {
    console.error("Websocket error: ", error);
  }

  const handleMove = (e) => {
    console.log(e)
  }
  
  const handleStop = (e) => {
    console.log(e)
  }

  return (
    <div className="App">
      <header className="App-header">
        <h2>Panda Control Interface</h2>
      <Joystick size={100} sticky={false} baseColor="Cyan" stickColor="black" move={handleMove} stop={handleStop}></Joystick>

        {/* <img src={logo} className="App-logo" alt="logo" />
        <p>
          Edit <code>src/App.js</code> and save to reload.
        </p>
        <a
          className="App-link"
          href="https://reactjs.org"
          target="_blank"
          rel="noopener noreferrer"
        >
          Learn React
        </a> */}
      </header>
    </div>
  );
}

export default App;
