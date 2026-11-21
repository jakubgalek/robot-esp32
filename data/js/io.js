"use strict";

// ======================================================================
//  Komunikacja z robotem
//  - komendy sterujące wysyłane NATYCHMIAST (ajax_inputs?cmd=1&...), odpowiedź "ok"
//  - telemetria (XML) odpytywana osobno co POLL_INTERVAL_MS, kolejne zapytanie
//    dopiero po odpowiedzi poprzedniego
//  - dopóki przycisk jazdy jest wciśnięty, co KEEPALIVE_MS leci keepalive;
//    firmware zatrzymuje silniki, gdy keepalive zniknie (deadman ~700 ms)
// ======================================================================
const POLL_INTERVAL_MS = 1000;
const POLL_TIMEOUT_MS = 8000;
const KEEPALIVE_MS = 200;

let AUTO_DRIVE_state = 0;   // stan z serwera (jedyne źródło prawdy dla przycisków)
let RADAR_state = 0;        // stan z serwera

const held = [false, false, false, false];   // MOTOR1..4 trzymane przez użytkownika

function sendCommand(params) {
    const query = Object.entries(params).map(([k, v]) => `${k}=${v}`).join('&');
    return fetch(`ajax_inputs?cmd=1&${query}&nocache=${Math.random() * 1000000}`, { cache: 'no-store' })
        .catch(() => {});
}

// ---------- Jazda: przyciski ----------
function setHeld(i, value) {
    if (held[i] === value) return;
    held[i] = value;
    const el = document.getElementById('MOTOR' + (i + 1));
    if (el) el.classList.toggle('pressed', value);
    sendCommand({ ['MOTOR' + (i + 1)]: value ? 1 : 0 });
}

function releaseAll() {
    for (let i = 0; i < 4; i++) setHeld(i, false);
}

function bindMotorButton(i) {
    const el = document.getElementById('MOTOR' + (i + 1));
    if (!el) return;
    el.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        try { el.setPointerCapture(e.pointerId); } catch (err) {}
        setHeld(i, true);
    });
    // pointerup/cancel/lostpointercapture dochodzą także, gdy palec/mysz zjedzie poza przycisk
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((type) =>
        el.addEventListener(type, () => setHeld(i, false)));
    el.addEventListener('contextmenu', (e) => e.preventDefault());
}

// Keepalive dla wciśniętych przycisków
setInterval(() => {
    const params = {};
    held.forEach((h, i) => { if (h) params['MOTOR' + (i + 1)] = 1; });
    if (Object.keys(params).length > 0) sendCommand(params);
}, KEEPALIVE_MS);

// Utrata fokusu / ukrycie karty => zwolnij wszystko
window.addEventListener('blur', releaseAll);
window.addEventListener('pagehide', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });

// ---------- Jazda: klawiatura (strzałki i klawiatura numeryczna 8/5/4/6) ----------
const keyToMotor = {
    ArrowUp: 0,    Numpad8: 0,
    ArrowDown: 1,  Numpad5: 1,
    ArrowLeft: 2,  Numpad4: 2,
    ArrowRight: 3, Numpad6: 3
};

document.addEventListener('keydown', (e) => {
    const i = keyToMotor[e.code];
    if (i === undefined) return;
    if (e.target && e.target.tagName === 'INPUT') return;   // nie przejmuj strzałek suwaka
    e.preventDefault();
    setHeld(i, true);
});

// Puszczenie klawisza ZATRZYMUJE ruch (wcześniej tylko zdejmowało podświetlenie)
document.addEventListener('keyup', (e) => {
    const i = keyToMotor[e.code];
    if (i === undefined) return;
    e.preventDefault();
    setHeld(i, false);
});

// ---------- Prędkość ----------
let sliderTimer = null;
function sendSpeed(value) {
    sendCommand({ Slider: value });
}

// ---------- Przyciski przełączane (stan pochodzi z serwera) ----------
function renderToggles() {
    const auto = document.querySelector('.button_automatic');
    if (auto) {
        auto.innerText = AUTO_DRIVE_state ? "Wyłącz auto jazdę" : "Automatyczna jazda";
        auto.classList.toggle('active', !!AUTO_DRIVE_state);
    }
    const radar = document.querySelector('.button_radar');
    if (radar) {
        radar.innerText = RADAR_state ? "Wyłącz radar" : "Uruchom radar";
        radar.classList.toggle('active', !!RADAR_state);
    }
}

function toggleAndRefresh(params) {
    // komenda -> po odpowiedzi natychmiast odśwież stan z serwera (bez lokalnego zgadywania)
    sendCommand(params).then(() => pollNow());
}

// ---------- Telemetria ----------
const textFields = ['voltageMotors', 'temp', 'altitude', 'voltageESP32', 'wifi', 'direction'];

function xmlText(xml, tag) {
    const node = xml.getElementsByTagName(tag)[0];
    return (node && node.childNodes[0]) ? node.childNodes[0].nodeValue : null;
}

function handleXml(xml) {
    textFields.forEach((tag) => {
        const value = xmlText(xml, tag);
        if (value === null) return;
        const elements = document.getElementsByClassName(tag);
        for (let k = 0; k < elements.length; k++) elements[k].innerHTML = value;
    });

    AUTO_DRIVE_state = (xmlText(xml, 'AUTO_DRIVE') === 'on') ? 1 : 0;
    RADAR_state = (xmlText(xml, 'RADAR') === 'on') ? 1 : 0;
    renderToggles();

    updateRadarData(xml);
}

let pollTimer = null;
let pollInFlight = false;

function schedulePoll(ms) {
    clearTimeout(pollTimer);
    pollTimer = setTimeout(pollNow, ms);
}

function pollNow() {
    clearTimeout(pollTimer);
    if (pollInFlight) { schedulePoll(200); return; }
    pollInFlight = true;

    const controller = new AbortController();
    const abortTimer = setTimeout(() => controller.abort(), POLL_TIMEOUT_MS);

    fetch(`ajax_inputs?nocache=${Math.random() * 1000000}`, { cache: 'no-store', signal: controller.signal })
        .then((r) => r.text())
        .then((text) => {
            const xml = new DOMParser().parseFromString(text, 'text/xml');
            handleXml(xml);
        })
        .catch((err) => console.warn("poll:", err))   // błąd sieci/parsowania - zobacz konsolę (F12)
        .finally(() => {
            clearTimeout(abortTimer);
            pollInFlight = false;
            schedulePoll(POLL_INTERVAL_MS);
        });
}

// ======================================================================
//  Mapa / radar
//  (UWAGA: przekształcenie punktów - odwrócony X, pominięty kąt robota -
//   zostanie przepisane razem z odometrią w kolejnym kroku)
// ======================================================================
let zoomLevel = 1;
let offsetX = 0;
let offsetY = 0;
let isPanning = false;
let startX, startY;

const svg = document.getElementById('radar-map');

let radarHistory = [];
const MAX_RADAR_POINTS = 20000;   // limit historii (wcześniej rosła bez końca i zamulała stronę)
let currentRobotPosition = { x: 0, y: 0, angle: 0 };
let lastRadarUpdateTime = 0;
const RADAR_UPDATE_INTERVAL = 100; // ms - minimalny czas między aktualizacjami radaru

let lastRadarRobotPose = null;

function updateRadarData(xml) {
    const robotXElement = xml.getElementsByTagName('robotX')[0];
    const robotYElement = xml.getElementsByTagName('robotY')[0];
    const robotAngleElement = xml.getElementsByTagName('robotAngle')[0];
    if (!robotXElement || !robotYElement || !robotAngleElement) return;

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

    // Jeśli robot stoi w miejscu – usuń poprzednie punkty z tej pozycji
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

    if (radarHistory.length > MAX_RADAR_POINTS) {
        radarHistory.splice(0, radarHistory.length - MAX_RADAR_POINTS);
    }

    requestDrawRadar();
}

// Rysowanie nie częściej niż raz na klatkę
let drawPending = false;
function requestDrawRadar() {
    if (drawPending) return;
    drawPending = true;
    requestAnimationFrame(() => {
        drawPending = false;
        drawRadar();
    });
}

function isValidDistance(d) {
    return Number.isFinite(d) && d > 0 && d < 1000;   // 1000 = brak echa
}

function drawRadar() {
    const svgPoints = document.getElementById('radar-points');
    const fragment = document.createDocumentFragment();

    radarHistory.forEach(point => {
        // Obliczamy pozycję punktu względem ORYGINALNEJ pozycji robota (z momentu skanowania)
        const totalAngle = (point.angle - 90) * (Math.PI / 180);

        const addPoint = (distance, sign) => {
            if (!isValidDistance(distance)) return;
            const x = sign * distance * Math.cos(totalAngle) + point.robotX;
            const y = sign * distance * Math.sin(totalAngle) + point.robotY;
            const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
            circle.setAttribute('cx', -x);
            circle.setAttribute('cy', y);
            circle.setAttribute('r', 3);
            circle.setAttribute('fill', 'orange');
            fragment.appendChild(circle);
        };

        addPoint(point.distance1, 1);
        addPoint(point.distance2, -1);
    });

    svgPoints.replaceChildren(fragment);
}

function updateRobotSVG(x, y, angle) {
    const robot = document.getElementById('robot');
    if (robot) {
        robot.setAttribute('transform', `translate(${x},${y}) rotate(${angle})`);

        const robotDirection = document.getElementById('robot-direction');
        if (robotDirection) {
            robotDirection.setAttribute('x2', 20 * Math.cos(-angle * Math.PI / 180));
            robotDirection.setAttribute('y2', 20 * Math.sin(-angle * Math.PI / 180));
        }
    }
}

// Funkcje zoomowania i panowania mapą
function zoomIn() {
    zoomLevel *= 1.2;
    updateViewBox();
}

function zoomOut() {
    zoomLevel /= 1.2;
    updateViewBox();
}

function updateViewBox() {
    const viewBoxSize = 1000 / zoomLevel;
    svg.setAttribute('viewBox', `${-viewBoxSize / 2 + offsetX} ${-viewBoxSize / 2 + offsetY} ${viewBoxSize} ${viewBoxSize}`);
    // punkty mają rozmiar w jednostkach viewBox, więc przeskalują się same - bez przerysowywania
}

// Przesuwanie myszą i palcem (pointer events)
function startPan(event) {
    isPanning = true;
    startX = event.clientX;
    startY = event.clientY;
    try { svg.setPointerCapture(event.pointerId); } catch (err) {}
    svg.style.cursor = 'grabbing';
}

function pan(event) {
    if (!isPanning) return;
    const rect = svg.getBoundingClientRect();
    const pixelsPerUnit = Math.min(rect.width, rect.height) / (1000 / zoomLevel);   // px na jednostkę viewBox
    offsetX -= (event.clientX - startX) / pixelsPerUnit;
    offsetY -= (event.clientY - startY) / pixelsPerUnit;
    startX = event.clientX;
    startY = event.clientY;
    updateViewBox();
}

function endPan() {
    isPanning = false;
    svg.style.cursor = 'grab';
}

// ======================================================================
//  Start
// ======================================================================
function init() {
    for (let i = 0; i < 4; i++) bindMotorButton(i);

    const rangeInput = document.getElementById('myRange');
    if (rangeInput) {
        // podczas przeciągania wysyłamy co najwyżej co 100 ms, a po puszczeniu wartość końcową
        rangeInput.addEventListener('input', function () {
            if (sliderTimer) return;
            const value = this.value;
            sliderTimer = setTimeout(() => { sliderTimer = null; sendSpeed(rangeInput.value); }, 100);
        });
        rangeInput.addEventListener('change', function () { sendSpeed(this.value); });
    }

    const autoButton = document.querySelector('.button_automatic');
    if (autoButton) autoButton.addEventListener('click', () => toggleAndRefresh({ AUTO_DRIVE: AUTO_DRIVE_state ? 0 : 1 }));

    const radarButton = document.querySelector('.button_radar');
    if (radarButton) radarButton.addEventListener('click', () => toggleAndRefresh({ RADAR: RADAR_state ? 0 : 1 }));

    if (svg) {
        svg.addEventListener('pointerdown', startPan);
        svg.addEventListener('pointermove', pan);
        svg.addEventListener('pointerup', endPan);
        svg.addEventListener('pointercancel', endPan);
    }

    renderToggles();
    pollNow();
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
