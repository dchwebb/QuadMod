let webserial;
let portButton;				// Connect/Disconnect button
let readingsSpan;			// DOM element where the incoming readings go
let initSettings = false;	// indicates when waiting for initial settings to be transmitted

document.addEventListener("DOMContentLoaded", setup);					// run the setup function when page is loaded

// get the DOM elements and assign listeners
function setup() 
{
	webserial = new WebSerialPort();
	if (webserial) {

		const textInput = document.getElementById("txt");				// user text input
		textInput.addEventListener("keyup", readTextInput);
  
		readingsSpan = document.getElementById("results");				// span for incoming serial messages

		webserial = new WebSerialPort();
		if (webserial) {
			webserial.on("data", serialRead);

			portButton = document.getElementById("btnConnect");			// connect/disconnect button
			portButton.addEventListener("click", openClosePort);
		}
	}
}


async function openClosePort() 
{
	let buttonLabel = "Connect";

	if (webserial.port) {
		await webserial.closePort();
	} else {
		await webserial.openPort();
		buttonLabel = "Disconnect";
	}
	portButton.innerHTML = buttonLabel;
}


function portOpened()
{
	setTimeout(()=>{ webserial.sendSerial("settings\r"); }, 500);		// Add some delay to the settings call as results were being truncated
	initSettings = true;
}


function serialRead(event) 
{
	let settings = event.detail.data;

	if (initSettings) {
		let data = settings.match("delay:(.*)\r");
		if (data != null) {
			document.getElementById("delayOnOff").checked = data[1] == "on" ? true : false;
		}

		data = settings.match("lfoSpeed:(.*)\r");
		if (data != null) {
			document.getElementById("lfoSpeed").value = parseInt(data[1]);
		}

		data = settings.match("lfoRange:(.*)\r");
		if (data != null) {
			document.getElementById("lfoRange").value = parseInt(data[1]);
		}

		data = settings.match("feedback:(.*)\r");
		if (data != null) {
			document.getElementById("feedback").value = parseInt(data[1]);
			initSettings = false;
		}
	}

	readingsSpan.innerHTML += settings;
}


function readTextInput(event) 
{
	// listen for the enter key (keyCode = 13) and transmit text field once found
	if (event.keyCode = 13) {
		webserial.sendSerial(event.target.value + '\r');
	}
}

function delayOnOff()
{
	webserial.sendSerial(`delay\r`);
}


function lfoSpeed() 
{
	let lfoSpeed = document.getElementById("lfoSpeed").value;
	webserial.sendSerial(`lfoSpeed:${lfoSpeed}\r`);
}


function lfoRange()
{
	let lfoRange = document.getElementById("lfoRange").value;
	webserial.sendSerial(`lfoRange:${lfoRange}\r`);
}

function feedback()
{
	let feedback = document.getElementById("feedback").value;
	webserial.sendSerial(`feedback:${feedback}\r`);
}


class WebSerialPort {
	constructor() 
	{
		if (!navigator.serial) {
			alert("WebSerial is not enabled in this browser");
			return false;
		}
		this.autoOpen = true;						// Auto connects if serial device restarted
		this.port;
		this.reader;
		this.serialReadPromise;

		this.incoming = {							// add an incoming data event
			data: null
		}

		this.dataEvent = new CustomEvent('data', {	// incoming serial data event
			detail: this.incoming,
			bubbles: true
		});

		navigator.serial.addEventListener("connect", this.serialConnect);
		navigator.serial.addEventListener("disconnect", this.serialDisconnect);

		// if the calling script passes in a message and handler add them as event listeners
		this.on = (message, handler) => {
			parent.addEventListener(message, handler);
		};
	}


	async openPort(thisPort) 
	{
		try {
			if (thisPort == null) {
				this.port = await navigator.serial.requestPort();		// if no port passed display pop-up window to select port
			} else {
				this.port = thisPort;
			}

			await this.port.open({ baudRate: 9600 });					// set port settings and open it
			this.serialReadPromise = this.listenForSerial();			// start the listenForSerial function
			console.log("Connected to",  thisPort);
			portOpened();
		} catch (err) {
			console.error("There was an error opening the serial port:", err);
		}
	}


	async closePort() 
	{
		if (this.port) {
			this.reader.cancel();				// stop the reader so the port can be closed
			await this.serialReadPromise;		// wait for the listenForSerial function to stop
			await this.port.close();			// close the serial port
			this.port = null;
		}
	}


	async sendSerial(data) 
	{
		if (this.port && this.port.writable) {
			const writer = this.port.writable.getWriter();					// initialize the writer
			var output = new TextEncoder().encode(data);					// convert the data to be sent to an array
			writer.write(output).then(writer.releaseLock());				// send it, then release the writer
		}
	}


	async listenForSerial() 
	{
		while (this.port && this.port.readable) {
			this.reader = this.port.readable.getReader();					// initialize the reader
			try {
				const { value, done } = await this.reader.read();			// read incoming serial buffer
				if (value) {
					this.incoming.data = new TextDecoder().decode(value);	// convert the input to a text string
					parent.dispatchEvent(this.dataEvent);					// fire the event
				}
				if (done) {
					break;
				}
			} catch (error) {
				console.log(error);
			} finally {
				this.reader.releaseLock();
			}
		}
	}

	// this event occurs every time a new serial device connects via USB
	serialConnect(event)
	{
		console.log(event.target);
		if (webserial.autoOpen) {
			webserial.openPort(event.target);
		}
	}

	// this event occurs every time a new serial device disconnects via USB
	serialDisconnect(event) {
		console.log(event.target);
	}
}