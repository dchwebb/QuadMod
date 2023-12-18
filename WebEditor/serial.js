// run the setup function when all the page is loaded:
document.addEventListener("DOMContentLoaded", setup);

// the DOM elements that might be changed by various functions:
let portButton;	 // the open/close port button
let readingsSpan; // DOM element where the incoming readings go
let webserial;

function setup() {
	// get the DOM elements and assign any listeners needed:
	webserial = new WebSerialPort();
	if (webserial) {
		// user text input:
		const textInput = document.getElementById("txt");
		textInput.addEventListener("keyup", readTextInput);
  
		// span for incoming serial messages:
		readingsSpan = document.getElementById("readings");

		webserial = new WebSerialPort();
		if (webserial) {
			webserial.on("data", serialRead);
			// port open/close button:
			portButton = document.getElementById("btnConnect");
			portButton.addEventListener("click", openClosePort);
		}
	}
}

async function openClosePort() {
	// label for the button will change depending on what you do:
	let buttonLabel = "Open port";
	// if port is open, close it; if closed, open it:
	if (webserial.port) {
		await webserial.closePort();
	} else {
		await webserial.openPort();
		buttonLabel = "Close port";
	}
	// change button label:
	portButton.innerHTML = buttonLabel;
}


function serialRead(event) {
	readingsSpan.innerHTML = event.detail.data;
}

function readTextInput(event) {
	// this function is triggered with every keystroke in the input field.
	// listen for the enter key (keyCode = 13) and skip the rest of
	// the function if you get any other key:
	if (event.keyCode != 13) {
		return;
	}
	// if you do get an enter keyCode, send the value of the field
	// out the serial port:
	webserial.sendSerial(event.target.value + '\r');
}






// need self = this for connect/disconnect functions
let self;

class WebSerialPort {
	constructor() {
		// if webserial doesn't exist, return false:
		if (!navigator.serial) {
			alert("WebSerial is not enabled in this browser");
			return false;
		}
		// TODO: make this an option.
		this.autoOpen = true;
		// copy this to a global variable so that
		// connect/disconnect can access it:
		self = this;

		// basic WebSerial properties:
		this.port;
		this.reader;
		this.serialReadPromise;
		// add an incoming data event:
		// TODO: data should probably be an ArrayBuffer or Stream
		this.incoming = {
			data: null
		}
		// incoming serial data event:
		this.dataEvent = new CustomEvent('data', {
			detail: this.incoming,
			bubbles: true
		});

		// TODO: bubble these up to calling script
		// so that you can change button names on 
		// connect or disconnect:
		navigator.serial.addEventListener("connect", this.serialConnect);
		navigator.serial.addEventListener("disconnect", this.serialDisconnect);

		// if the calling script passes in a message
		// and handler, add them as event listeners:
		this.on = (message, handler) => {
			parent.addEventListener(message, handler);
		};
	}

	async openPort(thisPort) {
		try {
			// if no port is passed to this function, 
			if (thisPort == null) {
				// pop up window to select port:
				this.port = await navigator.serial.requestPort();
			} else {
				// open the port that was passed:
				this.port = thisPort;
			}
			// set port settings and open it:
			// TODO: make port settings configurable
			// from calling script:
			await this.port.open({ baudRate: 9600 });
			// start the listenForSerial function:
			this.serialReadPromise = this.listenForSerial();

		} catch (err) {
			// if there's an error opening the port:
			console.error("There was an error opening the serial port:", err);
		}
	}

	async closePort() {
		if (this.port) {
			// stop the reader, so you can close the port:
			this.reader.cancel();
			// wait for the listenForSerial function to stop:
			await this.serialReadPromise;
			// close the serial port itself:
			await this.port.close();
			// clear the port variable:
			this.port = null;
		}
	}

	async sendSerial(data) {
		// if there's no port open, skip this function:
		if (!this.port) return;
		// if the port's writable: 
		if (this.port.writable) {
			// initialize the writer:
			const writer = this.port.writable.getWriter();
			// convert the data to be sent to an array:
			// TODO: make it possible to send as binary:
			var output = new TextEncoder().encode(data);
			// send it, then release the writer:
			writer.write(output).then(writer.releaseLock());
		}
	}

	async listenForSerial() {
		// if there's no serial port, return:
		if (!this.port) return;
		// while the port is open:
		while (this.port.readable) {
			// initialize the reader:
			this.reader = this.port.readable.getReader();
			try {
				// read incoming serial buffer:
				const { value, done } = await this.reader.read();
				if (value) {
					// convert the input to a text string:
					// TODO: make it possible to receive as binary:
					this.incoming.data = new TextDecoder().decode(value);

					// fire the event:
					parent.dispatchEvent(this.dataEvent);
				}
				if (done) {
					break;
				}
			} catch (error) {
				// if there's an error reading the port:
				console.log(error);
			} finally {
				this.reader.releaseLock();
			}
		}
	}

	// this event occurs every time a new serial device
	// connects via USB:
	serialConnect(event) {
		console.log(event.target);
		// TODO: make autoOpen configurable
		if (self.autoOpen) {
			self.openPort(event.target);
		}
	}

	// this event occurs every time a new serial device
	// disconnects via USB:
	serialDisconnect(event) {
		console.log(event.target);
	}
}