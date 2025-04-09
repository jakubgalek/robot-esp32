		strSlider = "";
		strMOTOR1 = "";
		strMOTOR2 = "";
		strMOTOR3 = "";
		strMOTOR4 = "";
		strAUTO_DRIVE = "";
		strRADAR = "";
		
		var MOTOR1_state = 0;
		var MOTOR2_state = 0;
		var MOTOR3_state = 0;
		var MOTOR4_state = 0;
		var AUTO_DRIVE_state = 0;
		var RADAR_state = 0;
		
		var x;
		var y;
		var speed;
		
		
var rangeInput = document.getElementById('myRange');
    
rangeInput.addEventListener('input', function() {
    var updatedValue = this.value;
	speed = updatedValue;
	sliderUpdate();
});
		
		function GetArduinoIO()
		{
			nocache = "&nocache=" + Math.random() * 1000000;
			var request = new XMLHttpRequest();
			request.onreadystatechange = function()
			{
				if (this.readyState == 4) {
					if (this.status == 200) {
						if (this.responseXML != null) {
							// XML file received - contains analog values, switch values and MOTOR states
							var count;
							// get voltageMotors
							var num_an = this.responseXML.getElementsByTagName('voltageMotors').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("voltageMotors")[count].innerHTML =
									this.responseXML.getElementsByTagName('voltageMotors')[count].childNodes[0].nodeValue;
							}
							// get temp
							var num_an = this.responseXML.getElementsByTagName('temp').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("temp")[count].innerHTML =
									this.responseXML.getElementsByTagName('temp')[count].childNodes[0].nodeValue;
							}
							// get altitude
							var num_an = this.responseXML.getElementsByTagName('altitude').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("altitude")[count].innerHTML =
									this.responseXML.getElementsByTagName('altitude')[count].childNodes[0].nodeValue;
							}
							// get voltageESP32
							var num_an = this.responseXML.getElementsByTagName('voltageESP32').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("voltageESP32")[count].innerHTML =
									this.responseXML.getElementsByTagName('voltageESP32')[count].childNodes[0].nodeValue;
							}
							// get wifi
							var num_an = this.responseXML.getElementsByTagName('wifi').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("wifi")[count].innerHTML =
									this.responseXML.getElementsByTagName('wifi')[count].childNodes[0].nodeValue;
							}							
							// get direction
							var num_an = this.responseXML.getElementsByTagName('direction').length;
							for (count = 0; count < num_an; count++) {
								document.getElementsByClassName("direction")[count].innerHTML =
									this.responseXML.getElementsByTagName('direction')[count].childNodes[0].nodeValue;
							}
							// MOTOR 1
							if (this.responseXML.getElementsByTagName('MOTOR')[0].childNodes[0].nodeValue === "on") {
								MOTOR1_state = 1;
							}
							else {
								MOTOR1_state = 0;
							}
							// MOTOR 2
							if (this.responseXML.getElementsByTagName('MOTOR')[1].childNodes[0].nodeValue === "on") {
								MOTOR2_state = 1;
							}
							else {
								MOTOR2_state = 0;
							}
							// MOTOR 3
							if (this.responseXML.getElementsByTagName('MOTOR')[2].childNodes[0].nodeValue === "on") {
								MOTOR3_state = 1;
							}
							else {
								MOTOR3_state = 0;
							}
							// MOTOR 4
							if (this.responseXML.getElementsByTagName('MOTOR')[3].childNodes[0].nodeValue === "on") {
								MOTOR4_state = 1;
							}
							else {
								MOTOR4_state = 0;
							}							
							// AUTOMATIC DRIVE
							if (this.responseXML.getElementsByTagName('AUTO_DRIVE')[0].childNodes[0].nodeValue === "on") {
								AUTO_DRIVE_state = 1;
							} else {
								AUTO_DRIVE_state = 0;
							}

							// RADAR
							if (this.responseXML.getElementsByTagName('RADAR')[0].childNodes[0].nodeValue === "on") {
								RADAR_state = 1;
							} else {
								RADAR_state = 0;
							}
								updateRadarData(this.responseXML); // Aktualizacja danych radarowych		
						}
					}
				}
			}
			// send HTTP GET request with MOTORs to switch on/off if any
			request.open("GET", "ajax_inputs?" + strMOTOR1 + strMOTOR2 + strMOTOR3 + strMOTOR4 + strAUTO_DRIVE + strRADAR + strSlider + nocache, true);
			request.send(null);
			setTimeout('GetArduinoIO()', 1000);
			strMOTOR1 = "";
			strMOTOR2 = "";
			strMOTOR3 = "";
			strMOTOR4 = "";
			strAUTO_DRIVE = "";
		
		}
		 
// Definiowanie kodów klawiszy
const up = 104;
const down = 101;
const left = 100;
const right = 102;

// Mapowanie kodów klawiszy na identyfikatory elementów
const keyMap = {
    [up]: "MOTOR1",
    [down]: "MOTOR2",
    [left]: "MOTOR3",
    [right]: "MOTOR4"
};

function activate(event) {
    const elementId = keyMap[event.keyCode];
    if (elementId) {
        const element = document.getElementById(elementId);
        
        // Ustawienie stylów dla elementu
        element.style.outline = '5px auto -webkit-focus-ring-color';
        element.style.boxShadow = 'inset 0px 0px 10px 1px rgb(0, 255, 157), 0px 0px 20px 5px rgb(0, 255, 157)';
        
        // Wywołanie odpowiedniej funkcji
        switch (elementId) {
            case 'MOTOR1': GetButton1up(); break;
            case 'MOTOR2': GetButton2up(); break;
            case 'MOTOR3': GetButton3up(); break;
            case 'MOTOR4': GetButton4up(); break;
        }
    }
}

function disactivate(event) {
    const elementId = keyMap[event.keyCode];
    if (elementId) {
        const element = document.getElementById(elementId);
        // Usuwa style boxShadow i outline
        element.style.boxShadow = '';
        element.style.outline = '';
        
        // Aby upewnić się, że nie ma pozostałości po stylach focus-ring
        element.blur(); 
    }
}

// Przypisz funkcje do zdarzeń
document.addEventListener('keydown', activate);
document.addEventListener('keyup', disactivate);


		function sliderUpdate()
		{
			strSlider = "&Slider=" + speed;
		}
		
		function GetButton1up()
		{
			if (MOTOR1_state === 0) {
				MOTOR1_state = 1;
				strMOTOR1 = "&MOTOR1=1";
			}
		}
		function GetButton1down()
		{
			if (MOTOR1_state === 1) {
				MOTOR1_state = 0;
				strMOTOR1 = "&MOTOR1=0";
			}
		}
		function GetButton2up()
		{
			if (MOTOR2_state === 0) {
				MOTOR2_state = 1;
				strMOTOR2 = "&MOTOR2=1";
			}
		}
		function GetButton2down()
		{
			if (MOTOR2_state === 1) {
				MOTOR2_state = 0;
				strMOTOR2 = "&MOTOR2=0";
			}
		}
		function GetButton3up()
		{
			if (MOTOR3_state === 0) {
				MOTOR3_state = 1;
				strMOTOR3 = "&MOTOR3=1";
			}
		}
		function GetButton3down()
		{
			if (MOTOR3_state === 1) {
				MOTOR3_state = 0;
				strMOTOR3 = "&MOTOR3=0";
			}
		}
		function GetButton4up()
		{
			if (MOTOR4_state === 0) {
				MOTOR4_state = 1;
				strMOTOR4 = "&MOTOR4=1";
			}
		}
		function GetButton4down()
		{
			if (MOTOR4_state === 1) {
				MOTOR4_state = 0;
				strMOTOR4 = "&MOTOR4=0";
			}
		}		
		function GetAutoDriveActive()
		{
			if (AUTO_DRIVE_state === 0) {
				AUTO_DRIVE_state = 1;
				strAUTO_DRIVE = "AUTO_DRIVE=1";
				let button = document.querySelector('.button_automatic');
				button.onclick = GetAutoDriveDisactivate;
				button.innerText = "Wyłącz auto jazdę";
				button.classList.add('active');
			}
		}
		function GetAutoDriveDisactivate()
		{
			if (AUTO_DRIVE_state === 1) {
				AUTO_DRIVE_state = 0;
				strAUTO_DRIVE = "&AUTO_DRIVE=0";
				let button = document.querySelector('.button_automatic');
				button.onclick = GetAutoDriveActive;
				button.innerText = "Automatyczna jazda";
				button.classList.remove('active');

			}
		}		
		function GetRadarActive()
		{
			if (RADAR_state === 0) {
				RADAR_state = 1;
				strRADAR = "RADAR=1";
				let button = document.querySelector('.button_radar');
				button.onclick = GetRadarDisactivate;
				button.innerText = "Wyłącz radar";
				button.classList.add('active');
			}
		}
		function GetRadarDisactivate()
		{
			if (RADAR_state === 1) {
				RADAR_state = 0;
				strRADAR = "&RADAR=0";
				let button = document.querySelector('.button_radar');
				button.onclick = GetRadarActive;
				button.innerText = "Uruchom radar";
				button.classList.remove('active');
			}
		}
		
		

let zoomLevel = 1;
let offsetX = 0;
let offsetY = 0;
let isPanning = false;
let startX, startY;

const svg = document.getElementById('radar-map');
svg.addEventListener('mousedown', startPan);
svg.addEventListener('mousemove', pan);
svg.addEventListener('mouseup', endPan);

let radarHistory = [];
let currentRobotPosition = { x: 0, y: 0, angle: 0 };
let lastRadarUpdateTime = 0;
const RADAR_UPDATE_INTERVAL = 100; // ms - minimalny czas między aktualizacjami radaru

let lastRadarRobotPose = null;

function updateRadarData(xml) {
const robotXElement = xml.getElementsByTagName('robotX')[0];
const robotYElement = xml.getElementsByTagName('robotY')[0];
const robotAngleElement = xml.getElementsByTagName('robotAngle')[0];

currentRobotPosition.x = parseFloat(robotXElement.textContent);
currentRobotPosition.y = parseFloat(robotYElement.textContent);
currentRobotPosition.angle = parseFloat(robotAngleElement.textContent);

updateRobotSVG(currentRobotPosition.x, currentRobotPosition.y, currentRobotPosition.angle);

// Radar wyłączony → nic nie rób
if (RADAR_state !== 1) return;

const now = Date.now();
if (now - lastRadarUpdateTime < RADAR_UPDATE_INTERVAL) return;
lastRadarUpdateTime = now;

const radarData = xml.getElementsByTagName('radarPoint');
if (radarData.length === 0) return;

const samePosition =
    lastRadarRobotPose &&
    currentRobotPosition.x === lastRadarRobotPose.x &&
    currentRobotPosition.y === lastRadarRobotPose.y &&
    currentRobotPosition.angle === lastRadarRobotPose.angle;

// Jeśli robot stoi w miejscu — usuń poprzednie punkty z tej pozycji
if (samePosition) {
    radarHistory = radarHistory.filter(
        point =>
            !(
                point.robotX === currentRobotPosition.x &&
                point.robotY === currentRobotPosition.y &&
                point.robotAngle === currentRobotPosition.angle
            )
    );
}

// Zapisz nową pozycję do porównań
lastRadarRobotPose = { ...currentRobotPosition };

// Dodaj nowe punkty
for (let i = 0; i < radarData.length; i++) {
    const angle = parseInt(radarData[i].getElementsByTagName('angle')[0].textContent);
    const distance1 = parseInt(radarData[i].getElementsByTagName('distance1')[0].textContent);
    const distance2 = parseInt(radarData[i].getElementsByTagName('distance2')[0].textContent);

    radarHistory.push({
        angle: angle,
        distance1: distance1,
        distance2: distance2,
        robotX: currentRobotPosition.x,
        robotY: currentRobotPosition.y,
        robotAngle: currentRobotPosition.angle,
        timestamp: now
    });
}

drawRadar();

}


function drawRadar() {
    const svg = document.getElementById('radar-points');
    svg.innerHTML = '';

    radarHistory.forEach(point => {
        // Obliczamy pozycję punktu względem ORYGINALNEJ pozycji robota (z momentu skanowania)
        const totalAngle = (point.angle - 90 + point.robotAngle) * (Math.PI / 180);
        
        const x1 = point.distance1 * Math.cos(totalAngle);
        const y1 = point.distance1 * Math.sin(totalAngle);
        const x2 = -point.distance2 * Math.cos(totalAngle);
        const y2 = -point.distance2 * Math.sin(totalAngle);

        // Punkty są rysowane względem pozycji robota Z MOMENTU SKANOWANIA
        const x1_final = x1 + point.robotX;
        const y1_final = y1 + point.robotY;
        const x2_final = x2 + point.robotX;
        const y2_final = y2 + point.robotY;

        // Rysowanie punktów
        const point1 = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        point1.setAttribute('cx', x1_final);
        point1.setAttribute('cy', -y1_final);
        point1.setAttribute('r', 3);
        point1.setAttribute('fill', 'orange');
        svg.appendChild(point1);

        const point2 = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        point2.setAttribute('cx', x2_final);
        point2.setAttribute('cy', -y2_final);
        point2.setAttribute('r', 3);
        point2.setAttribute('fill', 'orange');
        svg.appendChild(point2);
    });
}

function updateRobotSVG(x, y, angle) {
    const robot = document.getElementById('robot');
    if (robot) {
        robot.setAttribute('transform', `translate(${x},${-y}) rotate(${angle})`);
        
        const robotDirection = document.getElementById('robot-direction');
        if (robotDirection) {
            robotDirection.setAttribute('x2', 20 * Math.cos(-angle * Math.PI / 180));
            robotDirection.setAttribute('y2', 20 * Math.sin(-angle * Math.PI / 180));
        }
    }
}

// Funkcje zoomowania i panowania mapą (bez zmian)
function zoomIn() {
    zoomLevel *= 1.2;
    updateViewBox();
}

function zoomOut() {
    zoomLevel /= 1.2;
    updateViewBox();
}

function updateViewBox() {
    const svg = document.getElementById('radar-map');
    const viewBoxSize = 1000 / zoomLevel;
    svg.setAttribute('viewBox', `${-viewBoxSize / 2 + offsetX} ${-viewBoxSize / 2 + offsetY} ${viewBoxSize} ${viewBoxSize}`);
    drawRadar();
}

function startPan(event) {
    isPanning = true;
    startX = event.clientX;
    startY = event.clientY;
    document.getElementById('radar-map').style.cursor = 'grabbing';
}

function pan(event) {
    if (!isPanning) return;
    const dx = (event.clientX - startX) / zoomLevel;
    const dy = (event.clientY - startY) / zoomLevel;
    offsetX -= dx;
    offsetY -= dy;
    startX = event.clientX;
    startY = event.clientY;
    updateViewBox();
}

function endPan() {
    isPanning = false;
    document.getElementById('radar-map').style.cursor = 'grab';
}