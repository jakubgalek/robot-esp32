		strLED1 = "";
		strLED2 = "";
		strLED3 = "";
		strLED4 = "";
		var LED1_state = 0;
		var LED2_state = 0;
		var LED3_state = 0;
		var LED4_state = 0;
		var x;
		var y;
		
		
		function GetArduinoIO()
		{
			nocache = "&nocache=" + Math.random() * 1000000;
			var request = new XMLHttpRequest();
			request.onreadystatechange = function()
			{
				if (this.readyState == 4) {
					if (this.status == 200) {
						if (this.responseXML != null) {
							// XML file received - contains analog values, switch values and LED states
							var count;
							// get battery
							var num_an = this.responseXML.getElementsByTagName('battery').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("battery")[count].innerHTML =
									this.responseXML.getElementsByTagName('battery')[count].childNodes[0].nodeValue;
							}
							// get temp
							var num_an = this.responseXML.getElementsByTagName('temp').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("temp")[count].innerHTML =
									this.responseXML.getElementsByTagName('temp')[count].childNodes[0].nodeValue;
							}
							// get pressure
							var num_an = this.responseXML.getElementsByTagName('pressure').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("pressure")[count].innerHTML =
									this.responseXML.getElementsByTagName('pressure')[count].childNodes[0].nodeValue;
							}
							// get humidity
							var num_an = this.responseXML.getElementsByTagName('humidity').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("humidity")[count].innerHTML =
									this.responseXML.getElementsByTagName('humidity')[count].childNodes[0].nodeValue;
							}
							// get wifi
							var num_an = this.responseXML.getElementsByTagName('wifi').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("wifi")[count].innerHTML =
									this.responseXML.getElementsByTagName('wifi')[count].childNodes[0].nodeValue;
							}
							// get OX
							var num_an = this.responseXML.getElementsByTagName('OX').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByTagName("OX")[count].innerHTML =
									this.responseXML.getElementsByTagName('OX')[count].childNodes[0].nodeValue;
							}
							// get OY
							var num_an = this.responseXML.getElementsByTagName('OY').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByTagName("OY")[count].innerHTML =
									this.responseXML.getElementsByTagName('OY')[count].childNodes[0].nodeValue;
							}
							// get time
							var num_an = this.responseXML.getElementsByClassName("time").length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("time")[count].innerHTML =
									this.responseXML.getElementsByClassName("time")[count].childNodes[0].nodeValue;
							}
							// LED 1
							if (this.responseXML.getElementsByTagName('LED')[0].childNodes[0].nodeValue === "on") {
								LED1_state = 1;
							}
							else {
								LED1_state = 0;
							}
							// LED 2
							if (this.responseXML.getElementsByTagName('LED')[1].childNodes[0].nodeValue === "on") {
								LED2_state = 1;
							}
							else {
								LED2_state = 0;
							}
							// LED 3
							if (this.responseXML.getElementsByTagName('LED')[2].childNodes[0].nodeValue === "on") {
								LED3_state = 1;
							}
							else {
								LED3_state = 0;
							}
							// LED 4
							if (this.responseXML.getElementsByTagName('LED')[3].childNodes[0].nodeValue === "on") {
								LED4_state = 1;
							}
							else {
								LED4_state = 0;
							}
									
	
		x = parseInt(document.getElementById("myX").innerHTML);
		var circle = document.getElementById('cir');
		var ustaw=x;
		circle.setAttribute('cx', ustaw);
	
		y = parseInt(document.getElementById("myY").innerHTML);
		circle.setAttribute('cy', y);

		
						}
					}
				}
			}
			// send HTTP GET request with LEDs to switch on/off if any
			request.open("GET", "ajax_inputs?" + strLED1 + strLED2 + strLED3 + strLED4 + nocache, true);
			request.send(null);
			setTimeout('GetArduinoIO()', 300);
			strLED1 = "";
			strLED2 = "";
			strLED3 = "";
			strLED4 = "";
		}
		 
		 
		 const up=104;
		 const down=101;
		 const left=100;
		 const right=102;
		 
		 function activate(event)
		 {
			 if (event.keyCode===up)
			 {
				 GetButton1up();
				 //document.getElementById("LED1").style.boxShadow=shadow;
			 }
			 
			 if (event.keyCode===down)
			 {
				 GetButton2up();
			 }
			 
			 if (event.keyCode===left)
			 {
				 GetButton3up();
			 }
			 
			 if (event.keyCode===right)
			 {
				 GetButton4up();
			 }
		 }
		 
		 function disactivate(event)
		 {
			 if (event.keyCode===up)
			 {
				 GetButton1down();
			 }
			 
			 if (event.keyCode===down)
			 {
				 GetButton2down();
			 }
			 
			 if (event.keyCode===left)
			 {
				 GetButton3down();
			 }
			 
			 if (event.keyCode===right)
			 {
				 GetButton4down();
			 }
		 }		 
		 
		function GetButton1up()
		{
			if (LED1_state === 0) {
				LED1_state = 1;
				strLED1 = "&LED1=1";
			}
		}
		function GetButton1down()
		{
			if (LED1_state === 1) {
				LED1_state = 0;
				strLED1 = "&LED1=0";
			}
		}
		function GetButton2up()
		{
			if (LED2_state === 0) {
				LED2_state = 1;
				strLED2 = "&LED2=1";
			}
		}
		function GetButton2down()
		{
			if (LED2_state === 1) {
				LED2_state = 0;
				strLED2 = "&LED2=0";
			}
		}
		function GetButton3up()
		{
			if (LED3_state === 0) {
				LED3_state = 1;
				strLED3 = "&LED3=1";
			}
		}
		function GetButton3down()
		{
			if (LED3_state === 1) {
				LED3_state = 0;
				strLED3 = "&LED3=0";
			}
		}
		function GetButton4up()
		{
			if (LED4_state === 0) {
				LED4_state = 1;
				strLED4 = "&LED4=1";
			}
		}
		function GetButton4down()
		{
			if (LED4_state === 1) {
				LED4_state = 0;
				strLED4 = "&LED4=0";
			}
		}