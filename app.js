
(function(){
  'use strict';

  
  const $ = (id)=> document.getElementById(id);
  const on = (el, ev, fn)=> { if (el && el.addEventListener) el.addEventListener(ev, fn, false); };
  const fmt = (n,d=6)=> (isFinite(n)? Number(n).toFixed(d):'');
  const clamp = (v,min,max)=> Math.max(min, Math.min(max, v));

  function setBodyMode(val){ document.body && (document.body.dataset.coordMode = (val==='LL'?'LL':'MGRS')); }

  function toDateUTC(isoStr){ try { return new Date(isoStr); } catch(e){ return new Date(NaN);} }

  function nearestTimeIndex(timesIso, targetDate){
    if (!timesIso || !timesIso.length) return 0;
    let best=0, bestDt=Number.POSITIVE_INFINITY;
    for (let i=0;i<timesIso.length;i++){
      const ti = toDateUTC(timesIso[i]);
      const dt = Math.abs(ti - targetDate);
      if (dt < bestDt){ best=i; bestDt=dt; }
    }
    return best;
  }

/* MGRS 표기 포맷 */
  function formatMGRSDisplay(s){
    if (!s) return '';
    const t = s.replace(/\s+/g, '').toUpperCase();
    const idx = t.search(/[A-Z]/);
    if (idx < 0 || t.length < idx + 3) return s;
    const gzd  = t.slice(0, idx + 1);
    const grid = t.slice(idx + 1, idx + 3);
    const rest = t.slice(idx + 3);
    if (!rest) return `${gzd} ${grid}`;
    if (rest.length % 2 !== 0) return `${gzd} ${grid} ${rest}`;
    const half = rest.length / 2;
    const e = rest.slice(0, half);
    const n = rest.slice(half);
    return `${gzd} ${grid} ${e} ${n}`;
  }

  function toMGRS(lat, lon){
    try{
      if (window.mgrs){
        if (typeof mgrs.toMGRS === 'function') return mgrs.toMGRS([lon, lat], 5);
        if (typeof mgrs.toMgrs === 'function') return mgrs.toMgrs([lon, lat], 5);
        if (typeof mgrs.forward === 'function') return mgrs.forward([lon, lat], 5);
      }
    }catch(e){ console.warn('MGRS 변환 오류', e); }
    return '';
  }

  function fromMGRS(str){
    try{
      if (!str) return null;
      if (window.mgrs && typeof mgrs.toPoint === 'function'){
        const p = mgrs.toPoint(str.trim());
        return { lat: p[1], lon: p[0] };
      }
    }catch(e){ console.warn('MGRS 파싱 오류', e); }
    return null;
  }

/* 좌표 라벨 포맷 (위경도/MGRS) */
  function formatCoordLabel(lat, lon){
    const mode = (document.body?.dataset?.coordMode) || 'MGRS';
    if (mode === 'MGRS'){
      const m = toMGRS(lat, lon);
      if (m) return `MGRS ${formatMGRSDisplay(m)}`;
    }
    return `Lat ${fmt(lat,6)}, Lon ${fmt(lon,6)}`;
  }

  
/* 바람: 풍향/풍속 → U/V 벡터 */
  function dirSpeedToUV(dirDeg, speed) {
    const rad = (dirDeg || 0) * Math.PI / 180;
    const u = -speed * Math.sin(rad);
    const v = -speed * Math.cos(rad);
    return {u, v};
  }
/* 바람: U/V → 풍향/풍속 */
  function uvToDirSpeed(u, v) {
    const speed = Math.hypot(u, v);
    let deg = (Math.atan2(-u, -v) * 180 / Math.PI);
    if (deg < 0) deg += 360;
    return {dir: deg, speed};
  }
/* 바람 필드 2D 선형보간 */
  function bilinearUV(lon, lat, bbox, corners, hourIdx) {
    const {minLat, maxLat, minLon, maxLon} = bbox;
    const tx = (lon - minLon) / (maxLon - minLon);
    const ty = (lat - minLat) / (maxLat - minLat);
    const clamp01 = (x)=> Math.max(0, Math.min(1, x));
    const x = clamp01(tx), y = clamp01(ty);

    const nwU = corners.nw.u[hourIdx], nwV = corners.nw.v[hourIdx];
    const neU = corners.ne.u[hourIdx], neV = corners.ne.v[hourIdx];
    const swU = corners.sw.u[hourIdx], swV = corners.sw.v[hourIdx];
    const seU = corners.se.u[hourIdx], seV = corners.se.v[hourIdx];

    const topU = nwU*(1-x) + neU*x;
    const botU = swU*(1-x) + seU*x;
    const u = topU*(1-y) + botU*y;

    const topV = nwV*(1-x) + neV*x;
    const botV = swV*(1-x) + seV*x;
    const v = topV*(1-y) + botV*y;

    return {u, v};
  }
/* 시간 포함 보간 (바람) */
  function bilinearUVAtTime(lon, lat, grid, hourFloat){
    const base = (grid && typeof grid.baseIdx==='number') ? grid.baseIdx : 0;
    const abs = base + hourFloat;
    const i = Math.floor(abs);
    const f = Math.max(0, Math.min(1, abs - i));
    const i2 = Math.min(grid.corners.time.length - 1, i + 1);

    const uv_i  = bilinearUV(lon, lat, grid.bbox, grid.corners, i);
    const uv_i2 = bilinearUV(lon, lat, grid.bbox, grid.corners, i2);
    return {
      u: uv_i.u * (1 - f) + uv_i2.u * f,
      v: uv_i.v * (1 - f) + uv_i2.v * f
    };
  }
/* 풍하중 이동 시뮬레이션 한 스텝 진행 */
  function stepForward(lat,lon, bearingDeg, speedMS, dtSec){
    const R=6371000;
    const br = bearingDeg*Math.PI/180;
    const d = (speedMS*dtSec)/R;
    const lat1 = lat*Math.PI/180, lon1 = lon*Math.PI/180;
    const lat2 = Math.asin( Math.sin(lat1)*Math.cos(d) + Math.cos(lat1)*Math.sin(d)*Math.cos(br) );
    const lon2 = lon1 + Math.atan2(Math.sin(br)*Math.sin(d)*Math.cos(lat1), Math.cos(d)-Math.sin(lat1)*Math.sin(lat2));
    return {lat:lat2*180/Math.PI, lon:lon2*180/Math.PI};
  }
  const toCourse = (fromDeg)=> (fromDeg+180)%360;

  
  const map = L.map('map',{zoomControl:true}).setView([37.5665,126.9780], 8);


let hudAltEl = null, hudCoordEl = null;
let lastKnownElevation = NaN;
const HudControl = L.Control.extend({
  options: { position: 'bottomright' },
  onAdd: function() {
    const div = L.DomUtil.create('div', 'leaflet-control coord-hud');
    div.innerHTML = `
      <div class="hud-line" id="hudAlt">고도: —</div>
      <div class="hud-line" id="hudCoord">좌표: —</div>
    `;
    L.DomEvent.disableClickPropagation(div);
    L.DomEvent.disableScrollPropagation(div);
    setTimeout(()=>{
      hudAltEl   = document.getElementById('hudAlt');
      hudCoordEl = document.getElementById('hudCoord');
    },0);
    return div;
  }
});
map.addControl(new HudControl());

function updateHud(lat, lon, elevMeters, {loading=false}={}) {
  if (hudAltEl) {
    hudAltEl.textContent = '고도: ' + (loading
      ? '조회중…'
      : (isFinite(elevMeters) ? `${Math.round(elevMeters)} m` : '—'));
  }
  if (hudCoordEl) {
    hudCoordEl.textContent = formatCoordLabel(lat, lon);
  }
}

  
  let googleLayer = L.gridLayer.googleMutant({
    type: 'hybrid',
    maxZoom: 21
  }).addTo(map);

  L.control.scale().addTo(map);
  let osmLayer = null;



/* 지도 레이어 */
function setGoogleType(type){
  if (type === 'osm') {
    if (googleLayer && map.hasLayer(googleLayer)) map.removeLayer(googleLayer);
    if (!osmLayer) {
      osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
        maxZoom: 19
      });
    }
    if (!map.hasLayer(osmLayer)) osmLayer.addTo(map);
  } else {
    if (osmLayer && map.hasLayer(osmLayer)) map.removeLayer(osmLayer);
    
    if (typeof googleLayer.setMapType === 'function') {
      googleLayer.setMapType(type);
    } else if (typeof googleLayer.setMapTypeId === 'function') {
      googleLayer.setMapTypeId(type);
    } else {
      try { map.removeLayer(googleLayer); } catch(_) {}
      googleLayer = L.gridLayer.googleMutant({ type, maxZoom:21 });
      googleLayer.addTo(map);
    }
  }
  updateTypeButtons(type);
}

const TypeControl = L.Control.extend({
  options: { position: 'topright' },
  onAdd: function() {
const box = L.DomUtil.create('div', 'leaflet-control custom-box collapsed');
box.innerHTML = `
  <div class="fold-header" style="display: flex; align-items: center; justify-content: space-between; gap: 8px; width: max-content;">
    <span class="title" style="white-space: nowrap;">지도 모드</span>
    <span class="chev">▸</span>
  </div>
      <div class="fold-content">
        <div class="btn-row"><button class="btn" data-type="hybrid" title="위성 + 지명">위성 + 지명</button></div>
        <div class="btn-row"><button class="btn" data-type="satellite">위성</button></div>
        <div class="btn-row"><button class="btn" data-type="osm" title="오픈스트리트맵">OSM</button></div>


      </div>
    `;

//        <div class="btn-row"><button class="btn" data-type="roadmap">Roadmap</button></div>  //
//        <div class="btn-row"><button class="btn" data-type="terrain">Terrain</button></div>  //

    
    L.DomEvent.disableClickPropagation(box);
    L.DomEvent.disableScrollPropagation(box);
    const header = box.querySelector('.fold-header');
    L.DomEvent.on(header, 'click', (e)=>{
      L.DomEvent.stopPropagation(e);
      box.classList.toggle('collapsed');
    });

    
    box.querySelectorAll('button.btn[data-type]').forEach(btn=>{
      L.DomEvent.on(btn, 'click', (e)=>{
        L.DomEvent.stopPropagation(e); L.DomEvent.preventDefault(e);
        const t = e.currentTarget.getAttribute('data-type');
        setGoogleType(t);
      });
    });

    
const locBtn = box.querySelector('#btnLocate');
if (locBtn) { // if문으로 감싸서 버튼이 있을 때만 작동하게 합니다.
    L.DomEvent.on(locBtn, 'click', (e)=>{
      L.DomEvent.stopPropagation(e); L.DomEvent.preventDefault(e);
      goToMyLocation();
    });
}

const newLocBtn = document.getElementById('floatingLocateBtn');
if (newLocBtn) {
    newLocBtn.addEventListener('click', (e) => {
        e.preventDefault();
        e.stopPropagation();
        goToMyLocation();
    });
}
    return box;
  }
});
map.addControl(new TypeControl());


/* 지도 타입 버튼 UI 갱신 */
function updateTypeButtons(active){
  const node = document.querySelector('.leaflet-control.custom-box');
  if (!node) return;
  node.querySelectorAll('button.btn[data-type]').forEach(b=>{
    b.classList.toggle('active', b.getAttribute('data-type')===active);
  });
}
updateTypeButtons('hybrid');


  
  let myLocMarker=null, myLocCircle=null;
/* 내 위치로 이동 (Geolocation) */
  function goToMyLocation(){
    if (!navigator.geolocation){
      alert('이 브라우저는 위치 정보를 지원하지 않습니다.');
      return;
    }
    navigator.geolocation.getCurrentPosition(
      (pos)=>{
        const lat = pos.coords.latitude;
        const lon = pos.coords.longitude;
        const acc = pos.coords.accuracy || 0;


const TARGET_ZOOM = 11; 
 map.flyTo([lat, lon], TARGET_ZOOM, { animate: true, duration: 0.8 });
 
// === 사용자 요청 2: 지도에 내 위치 표시 ===
        const latlng = [lat, lon];
        
        // 기존 마커/원을 제거
        if (myLocMarker) map.removeLayer(myLocMarker);
        if (myLocCircle) map.removeLayer(myLocCircle);

        // 정확도 서클 추가
        myLocCircle = L.circle(latlng, acc, {
          color: '#1a73e8',
          fillColor: '#1a73e8',
          fillOpacity: 0.1,
          weight: 1
        }).addTo(map);

        // 위치 마커 추가
        myLocMarker = L.circleMarker(latlng, {
          radius: 6,
          weight: 2,
          color: '#ffffff',
          fillColor: '#1a73e8',
          fillOpacity: 1
        }).addTo(map);

        // 마커에 팝업 바인딩
        myLocMarker.bindPopup(`<b>내 위치</b><br>정확도: 약 ${Math.round(acc)}m`).openPopup();
// ==========================================

       },
       (err)=>{        if (location.protocol!=='https:' && location.hostname!=='localhost'){
          alert('내 위치 사용은 HTTPS(또는 localhost)에서만 동작합니다.');
        } else {
          alert('위치 정보 접근 실패: '+err.message);
        }
      },
      { enableHighAccuracy:true, maximumAge:10000, timeout:8000 }
    );
  }



  
  const drawLayer = L.featureGroup().addTo(map);
  const obsLayer = L.featureGroup().addTo(map);
  const lineLayer = L.featureGroup().addTo(map);
  const crossLayer = L.featureGroup().addTo(map);
  const driftLayer = L.layerGroup().addTo(map);
  let driftMinuteMarkers = [];
  const clickMarkers = L.layerGroup().addTo(map);

  
  const drawControl = new L.Control.Draw({
    edit: { featureGroup: drawLayer },
    draw: {
      polygon: { shapeOptions:{ color:'#4ade80', weight:2, fillColor:'#4ade80', fillOpacity:0.15 } },
      polyline: { shapeOptions:{ color:'#4ade80', weight:3 } },
      rectangle: { shapeOptions:{ color:'#4ade80', weight:2, fillColor:'#4ade80', fillOpacity:0.15 } },
      circle: { shapeOptions:{ color:'#4ade80', weight:2, fillColor:'#4ade80', fillOpacity:0.15 } },
      marker: true,
      circlemarker: false
    }
  });
  const defaultColor = '#4ade80';
  const defaultLabel = '도형';
  map.addControl(drawControl);
  map.on(L.Draw.Event.CREATED, (e)=>{
    const layer = e.layer;
    drawLayer.addLayer(layer);
    setTimeout(() => {
      applyStyleAndLabel(layer, defaultColor, defaultLabel);
      attachEditable(layer);
      if (typeof saveDrawnShapes === 'function') saveDrawnShapes();
    }, 0);
  });

  
  const coordMode=$('coordMode'), toggleLabels=$('toggleLabels');
  const obsLat=$('obsLat'), obsLon=$('obsLon'), obsMGRS=$('obsMGRS');

  const driftLat=$('driftLat'), driftLon=$('driftLon'), driftMGRS=$('driftMGRS');
  const driftMinutes=$('driftMinutes'), markerIntervalMin=$('markerIntervalMin'), stepSeconds=$('stepSeconds');
  const refreshSec=$('refreshSec'), accelFactor=$('accelFactor'), windSource=$('windSource');
  const runWindSim=$('runWindSim'), clearDrift=$('clearDrift'), pickStartBtn=$('pickStartBtn'), windMeta=$('windMeta');

  const obsName=$('obsName'), obsBearing=$('obsBearing'), lineKm=$('lineKm');
  const addObserverBtn=$('addObserverBtn'), recomputeX=$('recomputeX'), clearObs=$('clearObs');
  const observerList=$('observerList'), crossList=$('crossList');
  const clearShapes=$('clearShapes');

  const saveSnapshot=$('saveSnapshot'), loadSnapshot=$('loadSnapshot'), exportSnapshot=$('exportSnapshot'), importFile=$('importFile');

  
  const obsEditPanel=$('obsEditPanel');
  const editName=$('editName'), editBearing=$('editBearing'), editLineKm=$('editLineKm');
  const editMGRS=$('editMGRS'), editLat=$('editLat'), editLon=$('editLon');
  const editPick=$('editPick'), editSave=$('editSave'), editCancel=$('editCancel');

  
  let lastClickLatLng = null
  let observers=[];
  let selectedObserver=null;
  let removedIntersections=new Set();
  let reverseCache=new Map();
  let refreshTimer=null;

  
if (coordMode){
  setBodyMode(coordMode.value);
  on(coordMode,'change', ()=>{
    setBodyMode(coordMode.value);
    refreshMinuteMarkerTooltips();
    refreshObserverTooltips();
    refreshCrossTooltips();
    if (obsEditPanel && !obsEditPanel.hidden) fillEditPanel(selectedObserver);

    if (typeof updateHud === 'function' && lastClickLatLng){
      updateHud(
        lastClickLatLng.lat,
        lastClickLatLng.lng,
        (typeof lastKnownElevation !== 'undefined' ? lastKnownElevation : NaN)
      );
    }
  });
}

  on(toggleLabels,'change', ()=> refreshAllLabels());

  
  map.on('click', (e)=>{
    lastClickLatLng = e.latlng;
    const lat = e.latlng.lat, lon = e.latlng.lng;

    updateHud(lat, lon, NaN, {loading:true});
    if (window.google && google.maps) {
      window.__elevService = window.__elevService || new google.maps.ElevationService();
      window.__elevService.getElevationForLocations(
        { locations: [{ lat: lat, lng: lon }] },
        (results, status) => {
          let elev = NaN;
          if (status === 'OK' && results && results[0] && typeof results[0].elevation === 'number') {
            elev = results[0].elevation;
          }
          lastKnownElevation = elev;
          updateHud(lat, lon, elev);
        }
      );
    } else {
      updateHud(lat, lon, NaN);
    }

    clickMarkers.clearLayers();
    L.circleMarker([lat,lon],{radius:4,color:'#ff0000',weight:1,opacity:.8}).addTo(clickMarkers);

    if (obsLat) obsLat.value = fmt(lat);
    if (obsLon) obsLon.value = fmt(lon);
    if (driftLat) driftLat.value = fmt(lat);
    if (driftLon) driftLon.value = fmt(lon);

    const m = toMGRS(lat, lon);
    const pretty = formatMGRSDisplay(m);
    if (obsMGRS) obsMGRS.value = pretty;
    if (driftMGRS) driftMGRS.value = pretty;

    if (obsEditPanel && !obsEditPanel.hidden){
      if ((document.body.dataset.coordMode||'MGRS')==='LL'){
        if (editLat) editLat.value = fmt(lat);
        if (editLon) editLon.value = fmt(lon);
      } else {
        if (editMGRS) editMGRS.value = pretty;
      }
    }

    ensureClickPingStyles();
    const pingHtml = `
      <div class="click-ping-wrap">
        <div class="click-ping-ring"></div>
        <div class="click-ping-dot"></div>
      </div>
    `;
    const ping = L.marker(e.latlng, {
      icon: L.divIcon({ className: '', html: pingHtml, iconSize: [28, 28], iconAnchor: [14, 14] }),
      interactive: false
    }).addTo(clickMarkers);
    setTimeout(() => { try { clickMarkers.removeLayer(ping); } catch(_) {} }, 1000);
});


  
/* 관측점 편집창 열기 */
  function openObserverEditor(o){
    selectedObserver = o;
    if (!obsEditPanel) return;
    obsEditPanel.hidden = false;
    fillEditPanel(o);
  }
/* 편집창에 현재 관측점 값 채우기 */
  function fillEditPanel(o){
    if (!obsEditPanel || !o) return;
    if (editName) editName.value = o.name || '';
    if (editBearing) editBearing.value = o.bearing ?? 0;
    if (editLineKm) editLineKm.value = o.lineKm ?? 20;
    if ((document.body.dataset.coordMode||'MGRS')==='LL'){
      if (editLat) editLat.value = fmt(o.lat);
      if (editLon) editLon.value = fmt(o.lon);
    } else {
      if (editMGRS) editMGRS.value = formatMGRSDisplay(toMGRS(o.lat, o.lon));
    }
  }
/* 관측점 편집창 닫기 */
  function closeObserverEditor(){
    if (obsEditPanel) obsEditPanel.hidden = true;
    selectedObserver = null;
  }
  on(editPick,'click', ()=>{
    map.once('click', (e)=>{
      const {lat,lng} = e.latlng;
      if ((document.body.dataset.coordMode||'MGRS')==='LL'){
        if (editLat) editLat.value = fmt(lat);
        if (editLon) editLon.value = fmt(lng);
      } else {
        if (editMGRS) editMGRS.value = formatMGRSDisplay(toMGRS(lat,lng));
      }
    });
  });
  on(editCancel,'click', closeObserverEditor);
  on(editSave,'click', ()=>{
    if (!selectedObserver) return;
    let lat,lon;
    if ((document.body.dataset.coordMode||'MGRS')==='LL'){
      lat = parseFloat(editLat?.value);
      lon = parseFloat(editLon?.value);
    } else {
      const p = fromMGRS(editMGRS?.value); if (p){ lat=p.lat; lon=p.lon; }
    }
    if (!isFinite(lat)||!isFinite(lon)){ alert('좌표를 올바르게 입력하세요.'); return; }
    const brg = clamp(parseFloat(editBearing?.value)||0, -360, 360);
    const lk  = Math.max(0, parseFloat(editLineKm?.value)||20);
    const nm  = editName?.value || '관측';

    selectedObserver.name = nm;
    selectedObserver.lat  = lat;
    selectedObserver.lon  = lon;
    selectedObserver.bearing = brg;
    selectedObserver.lineKm  = lk;

    if (selectedObserver.marker){
      selectedObserver.marker.setLatLng([lat,lon]);
      bindObserverTooltip(selectedObserver.marker, selectedObserver);
      if (toggleLabels?.checked){ selectedObserver.marker.options._label = nm; }
    }
    drawObsLine(selectedObserver);
    renderObservers();
    recalcIntersections();
    closeObserverEditor();
  });

  
/* 관측점 툴팁 HTML 구성 */
  function getObserverTooltip(o){
    return `${o.name||'관측'} · ${formatCoordLabel(o.lat, o.lon)}`;
  }
/* 관측점 레이어에 툴팁 바인딩 */
  function bindObserverTooltip(marker, o){
    const txt = getObserverTooltip(o);
    const tt = marker.getTooltip && marker.getTooltip();
    if (tt && tt.setContent) tt.setContent(txt);
    else marker.bindTooltip(txt, { direction:'top', sticky:true });
  }
/* 관측점 툴팁 갱신 */
  function refreshObserverTooltips(){
    if (!Array.isArray(observers)) return;
    observers.forEach(o=>{ if (o.marker) bindObserverTooltip(o.marker, o); });
  }
/* 교차점 툴팁 바인딩 */
  function bindCrossTooltip(marker){
    const ll = marker.getLatLng();
    const txt = `교차점 · ${formatCoordLabel(ll.lat, ll.lng)}`;
    const tt = marker.getTooltip && marker.getTooltip();
    if (tt && tt.setContent) tt.setContent(txt);
    else marker.bindTooltip(txt, { direction:'top', sticky:true });
  }
/* 교차점 툴팁 갱신 */
  function refreshCrossTooltips(){
    crossLayer.eachLayer(m => bindCrossTooltip(m));
  }

/* 지도 클릭 위치 하이라이트 스타일 주입 */
  function ensureClickPingStyles(){
    if (document.getElementById('click-ping-styles')) return;
    const css = `
    @keyframes click-blink { 0%{opacity:0.2} 30%{opacity:1} 100%{opacity:0} }
    @keyframes click-ring  { 0%{ transform: translate(-50%, -50%) scale(0.5);  opacity:0.9 }
                             100%{ transform: translate(-50%, -50%) scale(1.8); opacity:0 } }
    .click-ping-wrap { position: relative; width:28px; height:28px; pointer-events:none; }
    .click-ping-dot  { position:absolute; left:50%; top:50%; transform:translate(-50%,-50%);
                       width:12px; height:12px; border-radius:50%;
                       background:#ffffff; border:2px solid #111; box-shadow:0 0 6px rgba(0,0,0,0.6);
                       animation: click-blink 900ms ease-out forwards; }
    .click-ping-ring { position:absolute; left:50%; top:50%; transform:translate(-50%,-50%);
                       width:20px; height:20px; border:2px solid #ffffff; border-radius:50%;
                       box-shadow:0 0 8px rgba(0,0,0,0.6);
                       transform-origin:center; animation: click-ring 900ms ease-out forwards; }`;
    const style=document.createElement('style'); style.id='click-ping-styles'; style.textContent=css; document.head.appendChild(style);
  }

/* 레이어 라벨 제거 유틸 */
function removeLabelForLayer(layer){
  if (layer && layer._labelMarker) {
    try { map.removeLayer(layer._labelMarker); } catch(_) {}
    layer._labelMarker = null;
  }
}
on(clearShapes,'click', ()=> {
  
  drawLayer.getLayers().forEach(removeLabelForLayer);
  drawLayer.clearLayers();
  if (typeof saveDrawnShapes === 'function') saveDrawnShapes();
});
map.on(L.Draw.Event.DELETED, (e)=>{
  e.layers.eachLayer(removeLabelForLayer);
});
drawLayer.on('layerremove', (e)=>{
  removeLabelForLayer(e.layer);
});


  
/* 관측점 추가 (입력/지도클릭) */
  function addObserver(o){
    const marker = L.marker([o.lat,o.lon],{draggable:true}).addTo(obsLayer);
    marker.bindTooltip(o.name||'관측', {permanent: !!(toggleLabels&&toggleLabels.checked), direction:'top', offset:[0,-10]});
    bindObserverTooltip(marker, o);
    marker.on('dragend', ()=>{
      const ll=marker.getLatLng(); o.lat=ll.lat; o.lon=ll.lng;
      bindObserverTooltip(marker, o);
      drawObsLine(o); renderObservers(); recalcIntersections();
    });
    marker.on('click', ()=> openObserverEditor(o));
    o.marker=marker;
    drawObsLine(o);
  }
/* 관측점에서 방위각/거리 라인 그리기 */
  function drawObsLine(o){
    if (o.line){ try{ lineLayer.removeLayer(o.line); }catch(_){ } o.line=null; }
    const distKm = Math.max(0, o.lineKm||20);
    const start=turf.point([o.lon,o.lat]);
    const end=turf.destination(start, distKm, o.bearing, {units:'kilometers'});
    const l=L.polyline([[o.lat,o.lon],[end.geometry.coordinates[1], end.geometry.coordinates[0]]], {color:'#4ade80',weight:2});
    l.addTo(lineLayer);
    l.on('click', ()=> openObserverEditor(o));
    if (toggleLabels&&toggleLabels.checked) l.bindTooltip(o.name||'방위선', {permanent:true, direction:'center'});
    o.line=l;
  }
/* 관측점 전체 렌더링 */
  function renderObservers(){
    const list = $('observerList');
    if (!list) return;
    list.innerHTML='';
    observers.forEach(o=>{
      const pill=document.createElement('div');
      pill.className='pill';
      pill.innerHTML=`<b>${o.name||'관측'}</b> · ${fmt(o.lat,4)}, ${fmt(o.lon,4)} · ${o.bearing}° · ${o.lineKm}km <span class="x" title="삭제">✕</span>`;
      pill.addEventListener('click', (ev)=>{ if (!ev.target.classList.contains('x')) openObserverEditor(o); });
      pill.querySelector('.x').onclick=()=>{
        try{ if (o.marker) obsLayer.removeLayer(o.marker); }catch(_){}
        try{ if (o.line) lineLayer.removeLayer(o.line); }catch(_){}
        observers = observers.filter(x=>x!==o);
        renderObservers(); recalcIntersections();
      };
      list.appendChild(pill);
    });
  }
  on($('addObserverBtn'),'click', ()=>{
    let lat,lon;
    if ((document.body.dataset.coordMode||'MGRS')==='LL'){
      lat=parseFloat($('obsLat').value); lon=parseFloat($('obsLon').value);
    }else{
      const p=fromMGRS($('obsMGRS').value);
      if (p){ lat=p.lat; lon=p.lon; }
    }
    if (!isFinite(lat)||!isFinite(lon)){ alert('좌표를 입력(또는 지도 클릭)하세요.'); return; }
    const bearing = clamp(parseFloat($('obsBearing').value)||0, -360, 360);
    const lk = Math.max(0, parseFloat($('lineKm').value)||20);
    const o={ id:Date.now()+Math.random(), name:($('obsName').value||'관측'), lat, lon, bearing, lineKm:lk, marker:null, line:null };
    observers.push(o); addObserver(o); renderObservers(); recalcIntersections();
  });
  on($('clearObs'),'click', ()=>{ obsLayer.clearLayers(); lineLayer.clearLayers(); observers=[]; renderObservers(); recalcIntersections(); });
  on($('recomputeX'),'click', ()=> recalcIntersections());

  
/* 역지오코딩 캐시 키 생성 */
  function reverseGeocodeKey(lat,lon){ return `${lat.toFixed(5)},${lon.toFixed(5)}`; }
  async function reverseGeocode(lat,lon){
    const key=reverseGeocodeKey(lat,lon);
    if (reverseCache.has(key)) return reverseCache.get(key);
    try{
      const res = await fetch(`https://nominatim.openstreetmap.org/reverse?format=jsonv2&lat=${lat}&lon=${lon}`, {headers:{'Accept-Language':'ko'}});
      const js = await res.json();
      const addr = js.display_name || '';
      reverseCache.set(key, addr);
      return addr;
    }catch(e){ return ''; }
  }

/* 관측선 간 교차점 계산 및 표시 */
  function recalcIntersections(){
    crossLayer.clearLayers();
    const list = $('crossList'); if (list) list.innerHTML='';
    const lines = observers.map(o=>{
      const start=turf.point([o.lon,o.lat]);
      const end=turf.destination(start, o.lineKm||20, o.bearing, {units:'kilometers'});
      return turf.lineString([[o.lon,o.lat], end.geometry.coordinates], {ref:o});
    });
    for(let i=0;i<lines.length;i++){
      for(let j=i+1;j<lines.length;j++){
        const isect = turf.lineIntersect(lines[i], lines[j]);
        if (isect && isect.features && isect.features.length){
          const pt = isect.features[0].geometry.coordinates;
          const lat=pt[1], lon=pt[0];
          const key=`${lat.toFixed(6)},${lon.toFixed(6)}`;
          if (removedIntersections.has(key)) continue;
          const m=L.marker([lat,lon]).addTo(crossLayer);
          bindCrossTooltip(m);
          (async ()=>{
            const addr = await reverseGeocode(lat,lon);
            m.bindPopup(`<b>교차점</b><br>${fmt(lat,6)}, ${fmt(lon,6)}<br>${addr}`);
          })();
          if (list){
            const item=document.createElement('div');
            item.className='pill';
            item.innerHTML=`<span>📍 ${fmt(lat,5)}, ${fmt(lon,5)}</span> <span class="x" title="다시 표시 안 함">숨김</span>`;
            item.querySelector('.x').onclick=()=>{ removedIntersections.add(key); recalcIntersections(); };
            list.appendChild(item);
          }
        }
      }
    }
  }
  on($('clearXRemoved'),'click', ()=>{ removedIntersections=new Set(); recalcIntersections(); });

  
/* 도형/마커 편집 가능 상태 연결 */
  function attachEditable(layer){
    layer.on('click', ()=> openStylePopup(layer, {name: layer.options && layer.options._label || '도형'}));
  }
/* 선택 레이어 스타일/라벨 팝업 */
  function openStylePopup(layer) {
    const currentColor = layer.options.color || '#4ade80';
    const currentFill = layer.options.fillColor || '#4ade80';
    const currentWeight = layer.options.weight || 2;
    const currentLabel = layer.options._label || '';
    const currentLabelColor = layer.options._labelColor || '#000000';
    const currentLabelBgColor = layer.options._labelBgColor || '#ffffff';

    const popupContent = `
      <div>
        <label>라벨 이름: <input type="text" id="labelText" value="${currentLabel}"></label><br>
        <label>글자 배경색: <input type="color" id="labelBgColor" value="${currentLabelBgColor}"></label><br>
        <label>글자 색상: <input type="color" id="labelColor" value="${currentLabelColor}"></label><br>
        <label>도형 색상: <input type="color" id="strokeColor" value="${currentColor}"></label><br>
        <label>선 굵기: <input type="number" id="strokeWeight" min="1" max="10" value="${currentWeight}"></label><br>
        <label>채움 색상: <input type="color" id="fillColor" value="${currentFill}"></label><br>
        <button id="applyStyle">적용</button>
      </div>
    `;

    layer.bindPopup(popupContent).openPopup();

    setTimeout(() => {
      const applyBtn = document.getElementById('applyStyle');
      applyBtn?.addEventListener('click', () => {
        const color = document.getElementById('strokeColor').value;
        const fill = document.getElementById('fillColor').value;
        const weight = parseInt(document.getElementById('strokeWeight').value, 10);
        const label = document.getElementById('labelText').value;
        const labelColor = document.getElementById('labelColor').value;
        const labelBgColor = document.getElementById('labelBgColor').value;

        layer.options.color = color;
        layer.options.fillColor = fill;
        layer.options.weight = weight;
        layer.options._label = label;
        layer.options._labelColor = labelColor;
        layer.options._labelBgColor = labelBgColor;

        applyStyleAndLabel(layer, color, label);

        if (typeof saveDrawnShapes === 'function') saveDrawnShapes();
        layer.closePopup();
      });
    }, 0);
  }
/* 레이어 라벨 추가 */
  function addLabelToLayer(layer, name) {
    if (!name || (!layer.getCenter && !layer.getLatLng)) return;
    const textColor = layer.options._labelColor || "#000000";
    const bgColor   = layer.options._labelBgColor || "white";
    const latlng = layer._labelLatLng
      ? L.latLng(layer._labelLatLng.lat, layer._labelLatLng.lng)
      : (layer.getCenter ? layer.getCenter() : layer.getLatLng());

    const label = L.marker(latlng, {
      icon: L.divIcon({
        className: 'custom-label',
        html: `<div class="label-box" style="color:${textColor};background-color:${bgColor}">${name}</div>`,
        iconAnchor: [name.length * 3.5, 10]
      }),
      draggable: true,
      interactive: true
    });

    if (layer._labelMarker) { map.removeLayer(layer._labelMarker); }
    layer._labelMarker = label;
    map.addLayer(label);

    layer.on('mouseover', () => { const el = label.getElement(); if (el) el.querySelector('.label-box')?.classList.add('hover'); });
    layer.on('mouseout',  () => { const el = label.getElement(); if (el) el.querySelector('.label-box')?.classList.remove('hover'); });

    label.on('dragend', function () {
      const newPos = label.getLatLng();
      layer._labelLatLng = { lat:newPos.lat, lng:newPos.lng };
      label.setLatLng(newPos);
      if (typeof saveDrawnShapes === 'function') saveDrawnShapes();
    });
  }
/* 스타일/라벨 적용 */
  function applyStyleAndLabel(layer, color, labelText) {
    try{
      layer.setStyle && layer.setStyle({ color, fillColor: color, weight: layer.options?.weight ?? 2, opacity:1, fillOpacity:0.5 });
    }catch(e){}
    layer.options = layer.options || {};
    layer.options._label = labelText;
    layer.options.color = color;
    if (toggleLabels && toggleLabels.checked && labelText){
      addLabelToLayer(layer, labelText);
    } else {
      if (layer._labelMarker) { map.removeLayer(layer._labelMarker); layer._labelMarker = null; }
    }
  }
/* 모든 라벨 갱신 */
  function refreshAllLabels(){
    const show = !!(toggleLabels && toggleLabels.checked);
/* 선택 레이어에 함수 적용 */
    function applyToLayer(layer){
      const label = layer.options && layer.options._label;
      if (label && show){
        addLabelToLayer(layer, label);
      } else {
        if (layer._labelMarker) {
          map.removeLayer(layer._labelMarker);
          layer._labelMarker = null;
        }
      }
    }
    [drawLayer, obsLayer, lineLayer].forEach(layerGroup => {
      layerGroup.eachLayer(applyToLayer);
    });
  }

  
  async function fetchWindSeriesPoint(lat, lon, level){
    const base=`https://api.open-meteo.com/v1/forecast`;
    const params10 = `latitude=${lat}&longitude=${lon}&hourly=windspeed_10m,winddirection_10m`;
    const params925 = `latitude=${lat}&longitude=${lon}&hourly=wind_speed_925hPa,wind_direction_925hPa`;
    const params850 = `latitude=${lat}&longitude=${lon}&hourly=wind_speed_850hPa,wind_direction_850hPa`;
    const params800 = `latitude=${lat}&longitude=${lon}&hourly=wind_speed_800hPa,wind_direction_800hPa`;
    const url = base + '?' + (level==='10m'?params10 : level==='925'?params925 : level==='850'?params850 : params800);
    const res = await fetch(url);
    const js = await res.json();
    const hr = js.hourly||{};
    let spd, dir;
    if (level==='10m'){ spd=hr.windspeed_10m; dir=hr.winddirection_10m; }
    else if (level==='925'){ spd=hr.wind_speed_925hPa; dir=hr.wind_direction_925hPa; }
    else if (level==='850'){ spd=hr.wind_speed_850hPa; dir=hr.wind_direction_850hPa; }
    else { spd=hr.wind_speed_800hPa; dir=hr.wind_direction_800hPa; }
    return {spd:spd||[], dir:dir||[], time:hr.time||[]};
  }
  async function fetchWindSeriesGrid(lat, lon, level, halfDeg=0.5){
    const nw = {lat: lat+halfDeg, lon: lon-halfDeg};
    const ne = {lat: lat+halfDeg, lon: lon+halfDeg};
    const sw = {lat: lat-halfDeg, lon: lon-halfDeg};
    const se = {lat: lat-halfDeg, lon: lon+halfDeg};

    const [nwS, neS, swS, seS] = await Promise.all([
      fetchWindSeriesPoint(nw.lat, nw.lon, level),
      fetchWindSeriesPoint(ne.lat, ne.lon, level),
      fetchWindSeriesPoint(sw.lat, sw.lon, level),
      fetchWindSeriesPoint(se.lat, se.lon, level),
    ]);

    const Lmin = Math.min(nwS.spd.length, neS.spd.length, swS.spd.length, seS.spd.length);
/* API 결과를 U/V 시계열로 변환 */
    function toUVSeries(s){
      const u=[], v=[];
      for (let i=0;i<Lmin;i++){
        const {u:uu, v:vv} = dirSpeedToUV(s.dir[i]||0, (s.spd[i]||0));
        u.push(uu); v.push(vv);
      }
      return {u,v};
    }

    const corners = {
      nw: {...nw, ...toUVSeries(nwS)},
      ne: {...ne, ...toUVSeries(neS)},
      sw: {...sw, ...toUVSeries(swS)},
      se: {...se, ...toUVSeries(seS)},
      time: (nwS.time||[]).slice(0,Lmin)
    };
    const bbox = { minLat: se.lat, maxLat: ne.lat, minLon: nw.lon, maxLon: ne.lon };
    return {corners, bbox};
  }
/* 이동경로(드리프트) 초기화 */
  function clearDriftPath(){
    driftLayer.clearLayers();
    const wm = $('windMeta'); if (wm) wm.textContent = '—';
    driftMinuteMarkers = [];
  }
  on(clearDrift,'click', clearDriftPath);
/* 분 단위 타임마커 툴팁 포맷 */
  function formatMinuteTooltip(minute, lat, lon){ return `${minute}분 · ${formatCoordLabel(lat, lon)}`; }
/* 타임마커 툴팁 갱신 */
  function refreshMinuteMarkerTooltips(){
    if (!Array.isArray(driftMinuteMarkers)) return;
    for (const mk of driftMinuteMarkers){
      if (!mk) continue;
      const ll = mk.getLatLng?.(); if (!ll) continue;
      const txt = formatMinuteTooltip(mk.__minute||0, ll.lat, ll.lng);
      const tt = mk.getTooltip && mk.getTooltip();
      if (tt && tt.setContent) tt.setContent(txt);
      else if (mk.bindTooltip) mk.bindTooltip(txt, { direction: 'top', sticky: true });
    }
  }
  on(pickStartBtn, 'click', () => {
    const once = (e) => {
      const { lat, lng } = e.latlng;
      if (driftLat)  driftLat.value  = fmt(lat);
      if (driftLon)  driftLon.value  = fmt(lng);
      const m = toMGRS(lat, lng);
      if (driftMGRS) driftMGRS.value = formatMGRSDisplay(m);
      map.off('click', once);
    };
    map.once('click', once);
  });
  on(runWindSim,'click', async ()=>{
    try{
      clearDriftPath();
      let lat,lon;
      if ((document.body.dataset.coordMode||'MGRS')==='LL'){
        lat=parseFloat(driftLat.value); lon=parseFloat(driftLon.value);
      }else{
        const p=fromMGRS(driftMGRS.value);
        if (p){ lat=p.lat; lon=p.lon; }
      }
      if (!isFinite(lat)||!isFinite(lon)){ alert('시작점을 입력/선택하세요.'); return; }

      const minutes = clamp(parseFloat(driftMinutes.value)||60, 1, 24*60);
      const intervalMin = clamp(parseFloat(markerIntervalMin.value)||10, 1, 120);
      const stepSec = clamp(parseFloat(stepSeconds.value)||60, 5, 3600);
      const accel = clamp(parseFloat(accelFactor.value)||1, 0.1, 10);
      const level = windSource.value;

      const grid = await fetchWindSeriesGrid(lat, lon, level, 0.5);
      if (!grid.corners.time.length) throw new Error('바람 데이터 없음');
      try{
        const now = new Date();
        const t0 = nearestTimeIndex(grid.corners.time, now);
        grid.baseIdx = t0;
      }catch(e){ grid.baseIdx = 0; }

      let rawLatLngs = [[lat, lon]];
      let rawTimes   = [0];
      let pts=[], t=0, curLat=lat, curLon=lon;
      const totalSec = minutes*60;

      while (t < totalSec){
        const hour = t / 3600;
        const {u, v} = bilinearUVAtTime(curLon, curLat, grid, hour);
        const {dir, speed} = uvToDirSpeed(u, v);
        const course = (dir + 180) % 360;
        const speedMS = (speed * accel) / 3.6;

        const dt = Math.min(stepSec, totalSec - t);
        const n = stepForward(curLat, curLon, course, speedMS, dt);
        curLat = n.lat; curLon = n.lon;
        t += dt;

        pts.push([curLat, curLon]);
        rawLatLngs.push([curLat, curLon]);
        rawTimes.push(t);
      }

      let smoothLatLngs = [[lat,lon]].concat(pts);
      try{
        const line = turf.lineString([[lon,lat]].concat(pts.map(p=>[p[1], p[0]])));
        const smooth = turf.bezierSpline(line, {resolution: 2000, sharpness: 0.85});
        smoothLatLngs = smooth.geometry.coordinates.map(([x,y])=>[y,x]);
      }catch(e){}

      L.polyline(smoothLatLngs, {color:'#4ade80', weight:3, opacity:0.95}).addTo(driftLayer);

      for (let m = intervalMin; m <= minutes; m += intervalMin) {
        const targetSec = m * 60;
        let k = rawTimes.findIndex((tt, idx) =>
          idx < rawTimes.length - 1 && tt <= targetSec && targetSec <= rawTimes[idx + 1]
        );
        if (k < 0) k = rawTimes.length - 2;
        const t0 = rawTimes[k], t1 = rawTimes[k + 1];
        const r = (t1 === t0) ? 0 : (targetSec - t0) / (t1 - t0);

        const A = turf.point([rawLatLngs[k][1],     rawLatLngs[k][0]]);
        const B = turf.point([rawLatLngs[k+1][1],   rawLatLngs[k+1][0]]);
        const brg = turf.bearing(A, B);
        const segKm = turf.distance(A, B, {units: 'kilometers'});
        const R = turf.destination(A, segKm * r, brg, {units: 'kilometers'});
        const markerLatLng = [R.geometry.coordinates[1], R.geometry.coordinates[0]];

        const _mm = L.circleMarker(markerLatLng, {
          radius: 5, weight: 2, color: '#0f172a', fillColor: '#ffffff', fillOpacity: 1
        }).addTo(driftLayer);
        _mm.__minute = m;
        _mm.bindTooltip(`${m}분 · ${formatCoordLabel(markerLatLng[0], markerLatLng[1])}`, { direction: 'top', sticky: true });
        driftMinuteMarkers.push(_mm);
      }

      if (windMeta){
        const {u, v} = (function(){
          const i=0;
          const mU=(grid.corners.nw.u[i]+grid.corners.ne.u[i]+grid.corners.sw.u[i]+grid.corners.se.u[i])/4;
        const mV=(grid.corners.nw.v[i]+grid.corners.ne.v[i]+grid.corners.sw.v[i]+grid.corners.se.v[i])/4;
          return {u:mU, v:mV};
        })();
        const first = uvToDirSpeed(u,v);
        const c0 = (first.dir+180)%360;
        windMeta.textContent = `소스:${level} · 첫 시각 평균풍속:${(first.speed||0).toFixed(1)}km/h (From ${first.dir.toFixed(0)}° → 진행 ${c0.toFixed(0)}°)`;
      }

      const rsec = Math.max(0, parseInt(refreshSec.value||'0',10));
      if (refreshTimer){ clearInterval(refreshTimer); refreshTimer=null; }
      if (rsec>0){ refreshTimer = setInterval(()=> runWindSim.click(), rsec*1000); }
    }catch(err){
      if (windMeta) windMeta.textContent='오류: '+err.message;
    }
  });

  
  on(clearShapes,'click', ()=> { drawLayer.clearLayers(); if (typeof saveDrawnShapes === 'function') saveDrawnShapes(); });

  
/* 앱 상태 스냅샷 저장 */
  function snapshot(){
    return {
      coordMode: document.body.dataset.coordMode||'MGRS',
      observers: observers.map(o=>({name:o.name, lat:o.lat, lon:o.lon, bearing:o.bearing, lineKm:o.lineKm})),
      removedX: Array.from(removedIntersections),
      reverseCache: Array.from(reverseCache.entries())
    };
  }
/* 스냅샷에서 복원 */
  function restore(s){
    setBodyMode(s.coordMode||'MGRS');
    const modeSel = $('coordMode'); if (modeSel) modeSel.value = s.coordMode||'MGRS';
    obsLayer.clearLayers(); lineLayer.clearLayers(); observers=[];
    (s.observers||[]).forEach(o=>{ const copy={...o, marker:null, line:null}; observers.push(copy); addObserver(copy); });
    renderObservers();
    removedIntersections = new Set(s.removedX||[]);
    reverseCache = new Map(s.reverseCache||[]);
    recalcIntersections();
    refreshAllLabels();
  }
  on(saveSnapshot,'click', ()=>{ localStorage.setItem('snapshot_v1', JSON.stringify(snapshot())); alert('저장 완료'); });
  on(loadSnapshot,'click', ()=>{
    const raw = localStorage.getItem('snapshot_v1');
    if (!raw) return alert('저장된 스냅샷이 없습니다.');
    try{ restore(JSON.parse(raw)); }catch(e){ alert('불러오기 실패: '+e.message); }
  });
  on(exportSnapshot,'click', ()=>{
    const s = snapshot();
    const blob = new Blob([JSON.stringify(s,null,2)], {type:'application/json'});
    const a=document.createElement('a'); a.href=URL.createObjectURL(blob); a.download='snapshot.json'; a.click(); URL.revokeObjectURL(a.href);
  });
  on(importFile,'change', (e)=>{
    const f=e.target.files[0]; if (!f) return;
    const rd=new FileReader(); rd.onload=()=>{ try{ restore(JSON.parse(rd.result)); alert('가져오기 완료'); }catch(err){ alert('가져오기 실패: '+err.message); } }; rd.readAsText(f);
  });

  
  ;['closeEditBtn','saveEditBtn'].forEach(id=>{ const el=$(id); on(el,'click', (ev)=>ev.preventDefault()); });

  
  window.map = map;
  window.drawLayer = drawLayer;
  window.applyStyleAndLabel = applyStyleAndLabel;
  window.attachEditable = attachEditable;
  window.refreshAllLabels = refreshAllLabels;

})();


/* 그려진 도형 저장(LocalStorage) */
function saveDrawnShapes(){
  if (!window.drawLayer) return;
  const features = [];
  window.drawLayer.eachLayer(layer => {
    const geojson = layer.toGeoJSON();
    geojson.properties = {
      _label: layer.options._label || '',
      color: layer.options.color || '#4ade80',
      fillColor: layer.options.fillColor || '#4ade80',
      weight: layer.options.weight || 2,
      _labelColor: layer.options._labelColor || '#000000',
      _labelBgColor: layer.options._labelBgColor || '#ffffff',
      labelLatLng: layer._labelMarker ? layer._labelMarker.getLatLng() : (layer._labelLatLng || null)
    };
    features.push(geojson);
  });
  localStorage.setItem('drawnShapes', JSON.stringify(features));
}
/* 저장된 도형 로드 */
function loadDrawnShapes(){
  if (!window.drawLayer || !window.map || typeof window.applyStyleAndLabel !== 'function' || typeof window.attachEditable !== 'function') return;
  const raw = localStorage.getItem('drawnShapes');
  if (!raw) return;
  try {
    const features = JSON.parse(raw);
    features.forEach(f => {
      const layer = L.GeoJSON.geometryToLayer(f);
      layer.options._label = f.properties._label;
      layer.options.color = f.properties.color;
      layer.options.fillColor = f.properties.fillColor;
      layer.options.weight = f.properties.weight;
      layer.options._labelColor = f.properties._labelColor || '#000000';
      layer.options._labelBgColor = f.properties._labelBgColor || '#ffffff';
      if (f.properties.labelLatLng) {
        layer._labelLatLng = { lat:f.properties.labelLatLng.lat, lng:f.properties.labelLatLng.lng };
      }
      window.applyStyleAndLabel(layer, layer.options.color, layer.options._label, layer.options.weight);
      window.attachEditable(layer);
      window.drawLayer.addLayer(layer);

      if (layer._labelLatLng && layer._labelMarker) {
        layer._labelMarker.setLatLng(layer._labelLatLng);
      }
    });
  } catch(e) { console.error('도형 불러오기 오류:', e); }
}
if (window.map && window.drawLayer){
  window.map.on(L.Draw.Event.CREATED, () => saveDrawnShapes());
  window.drawLayer.on('layerremove', () => saveDrawnShapes());
  window.drawLayer.on('layeradd', () => saveDrawnShapes());
  window.map.whenReady(() => {
    loadDrawnShapes();
    if (typeof window.refreshAllLabels === 'function') window.refreshAllLabels();
  });
}


(function ensureAsideScrollWrapper(){
  const aside = document.querySelector('aside');
  if (!aside) return;

  
  if (aside.querySelector('.aside-scroll')) return;

  
  const toggleBtn = document.getElementById('asideToggle');

  
  const wrap = document.createElement('div');
  wrap.className = 'aside-scroll';

  
  const children = Array.from(aside.childNodes);
  for (const node of children){
    if (toggleBtn && node === toggleBtn) continue;
    wrap.appendChild(node);
  }

  
  if (toggleBtn) aside.insertBefore(wrap, toggleBtn);
  else aside.appendChild(wrap);
})();


(function initSidebarHeaders(){
  document.querySelectorAll('aside .pane').forEach(p=>{
    const h = p.querySelector('h2'); if(!h) return;
    
    const raw = h.textContent || "";
    const clean = raw.replace(/^[^\w가-힣]+/, "").trim();
    h.textContent = clean;

    
    if (!p.dataset.bindCollapse){
      h.addEventListener('click', ()=> p.classList.toggle('collapsed'));
      p.dataset.bindCollapse = "1";
    }
  });
})();


(function bindSidebarToggle(){
  const hideBtn = document.getElementById('asideToggle');
  const showBtn = document.getElementById('asideExpand');

  hideBtn && hideBtn.addEventListener('click', ()=>{
    document.body.classList.add('sidebar-collapsed');
  });

  showBtn && showBtn.addEventListener('click', ()=>{
    document.body.classList.remove('sidebar-collapsed');
  });
})();


  // 토글 버튼 클릭 핸들러: 현재 상태를 반전시킵니다.
  function toggleCollapsed() {
      applyCollapsed(!STATE.isCollapsed);
  }

/* =========================
   모바일 사이드바 보정 모듈
   ========================= */
(function(){
  const STATE = {
    el: null,
    isCollapsed: true,
    startX: 0,
    startY: 0,
    moved: false,
    touchFromEdge: false,
    touchOnSidebar: false,
  };

  // 사이드바 후보를 자동 탐색 (기존 구조 변경하지 않음)
  function findSidebarElement(){
    const candidates = [
      '[data-role="sidebar"]',
      'aside',
      '.sidebar',
      '.side-panel',
      '.drawer',
      '.nav-panel',
      '.panel-left',
      '.left-side',
    ];
    for (const sel of candidates){
      const el = document.querySelector(sel);
      if (!el) continue;
      // 화면의 왼쪽에 붙어 있고(또는 고정) 지도 옆을 차지하는 큰 패널일 확률이 높음
      const rect = el.getBoundingClientRect();
      if (rect.width > 120 && rect.x < window.innerWidth * 0.4){
        return el;
      }
    }
    return null;
  }

  // 모바일에서 보기 좋게 사이즈/폰트 조정 (폭과 글자 살짝 줄임)
  function applyMobileSizing(){
    if (!STATE.el) return;
    const isMobile = window.innerWidth <= 768;
    if (isMobile){
      // 스타일을 "추가"만 하고, 기존 인라인/클래스는 건드리지 않음
      STATE.el.classList.add('js-mobile-sidebar-animate');
      // 폭: 글자 길이에 따라 자연스럽게 보이도록 상한/하한
      const targetWidthVW = 62; // 필요하면 55~70 사이 미세조정 가능
      STATE.el.style.width = `min(${targetWidthVW}vw, ${Math.max(280, Math.min(window.innerWidth * 0.9, 420))}px)`;
      STATE.el.style.maxWidth = '85vw';
      STATE.el.style.minWidth = '240px';
      STATE.el.style.fontSize = '0.95em'; // 살짝만 축소
      STATE.el.style.transformOrigin = 'left center';
      // 접힘 상태 반영
      applyCollapsed(STATE.isCollapsed, /*animate*/false);
    } else {
      // 데스크톱은 우리 보정 해제 (기존 스타일을 따르게)
      STATE.el.classList.remove('js-mobile-sidebar-animate');
      STATE.el.style.removeProperty('width');
      STATE.el.style.removeProperty('max-width');
      STATE.el.style.removeProperty('min-width');
      STATE.el.style.removeProperty('font-size');
      STATE.el.style.removeProperty('transform');
    }
  }

function applyCollapsed(collapsed) {
  if (!STATE.el) return;
  STATE.isCollapsed = !!collapsed;

  if (STATE.isCollapsed) {
    STATE.el.classList.add('collapsed');         // 사이드바 자체에 클래스 추가
    document.body.classList.add('sidebar-collapsed'); // ★ body에도 클래스 추가 (레이아웃 변경용)
  } else {
    STATE.el.classList.remove('collapsed');
    document.body.classList.remove('sidebar-collapsed');
  }

  // 지도가 있다면 크기 재조정 (사이드바가 사라지면 지도가 넓어져야 하므로)
  if (window.map) {
    setTimeout(() => window.map.invalidateSize(), 350); 
  }
}

  // 외부에서 호출 가능하도록(필요시)
  window.__mobileSidebar = {
    collapse: ()=>applyCollapsed(true),
    expand:   ()=>applyCollapsed(false),
    toggle:   ()=>applyCollapsed(!STATE.isCollapsed),
  };

  // 스와이프 제스처
  function onTouchStart(ev){
    if (!STATE.el || window.innerWidth > 768) return;
    const t = ev.touches ? ev.touches[0] : ev;
    STATE.startX = t.clientX;
    STATE.startY = t.clientY;
    STATE.moved = false;

    const sbRect = STATE.el.getBoundingClientRect();
    STATE.touchFromEdge = STATE.startX < 20 && STATE.isCollapsed; // 화면 왼쪽 에지에서 시작 → 펼치기 제스처
    STATE.touchOnSidebar = (
      STATE.startX >= sbRect.left &&
      STATE.startX <= sbRect.right &&
      STATE.startY >= sbRect.top &&
      STATE.startY <= sbRect.bottom
    );

    
  }

  function onTouchMove(ev){
    if (!STATE.el || window.innerWidth > 768) return;
    const t = ev.touches ? ev.touches[0] : ev;
    const dx = t.clientX - STATE.startX;
    const dy = t.clientY - STATE.startY;
    if (!STATE.moved && Math.hypot(dx, dy) > 8) STATE.moved = true;

    // 사이드바를 왼쪽/오른쪽으로 따라 움직이는 미리보기(옵션)
    if (STATE.touchOnSidebar){
      // 세로 스크롤보다 가로 이동이 크면 우리 제어
      if (Math.abs(dx) > Math.abs(dy) * 1.2){
        ev.preventDefault();
        const w = STATE.el.getBoundingClientRect().width || 320;
        let tx = Math.min(0, Math.max(-w + 24, dx * 0.6 * (STATE.isCollapsed ? 0 : 1) - (STATE.isCollapsed ? 0 : 0)));
        // 접힌 상태에서 오른쪽으로 드래그하면 펼치기 쪽으로
        if (STATE.isCollapsed && dx > 0) tx = Math.min(0, -w + 24 + dx);
        STATE.el.style.transform = `translateX(${tx}px)`;
      }
    }
  }

  function onTouchEnd(ev){
    if (!STATE.el || window.innerWidth > 768) return;
    const t = (ev.changedTouches && ev.changedTouches[0]) || (ev.touches && ev.touches[0]);
    const endX = t ? t.clientX : STATE.startX;
    const endY = t ? t.clientY : STATE.startY;
    const dx = endX - STATE.startX;
    const dy = endY - STATE.startY;

    const H = Math.abs(dx) > Math.abs(dy) * 1.2; // 수평 제스처 우선
    const THRESH = 60; // 트리거 임계값(px)

    // 1) 접힌 상태에서 좌측 에지 스와이프 → 펼치기
    if (STATE.touchFromEdge && H && dx > THRESH){
      applyCollapsed(false);
      return;
    }

    // 2) 열린 상태에서 사이드바 위에서 왼쪽 스와이프 → 접기
    if (STATE.touchOnSidebar && H && dx < -THRESH && !STATE.isCollapsed){
      applyCollapsed(true);
      return;
    }
  }

  function init(){
    // 사이드바 탐색
    STATE.el = findSidebarElement();
    if (!STATE.el) return; // 사이드바가 없는 레이아웃이면 아무것도 안 함

    applyMobileSizing();

    let lastWidth = window.innerWidth; 

window.addEventListener('resize', () => {
  const currentWidth = window.innerWidth;
  
  if (currentWidth === lastWidth) return;
  
  lastWidth = currentWidth;
  applyMobileSizing();
});

    // === 사용자 요청 5: 버튼 클릭 이벤트 리스너 추가 ===
    on($('asideToggle'), 'click', toggleCollapsed);
    on($('asideExpand'), 'click', toggleCollapsed);
    // ===============================================

    // 초기: 모바일이면 반쯤 접힌 상태로 시작해도 좋음 (원하면 false로)
    if (window.innerWidth <= 768){
      applyCollapsed(true, false);
    }
  }

  // DOM 준비 후 초기화
  if (document.readyState === 'loading'){
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
