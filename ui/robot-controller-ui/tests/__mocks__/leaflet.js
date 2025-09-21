// tests/__mocks__/leaflet.js
const noop = () => {};
const ret = function () { return this; };

const mapObj = {
  setView: jest.fn(ret),
  addLayer: jest.fn(ret),
  remove: jest.fn(noop),
  on: jest.fn(ret),
  off: jest.fn(ret),
  addControl: jest.fn(ret),
  invalidateSize: jest.fn(ret),
  getSize: jest.fn(() => ({ x: 800, y: 600 })),
  getCenter: jest.fn(() => ({ lat: 34.05, lng: -118.24 })),
  flyTo: jest.fn(ret),
  fitBounds: jest.fn(ret),
};

function makeLayer(extra = {}) {
  return {
    addTo: jest.fn(ret),      // <-- returns itself, not the map
    remove: jest.fn(noop),
    ...extra,
  };
}

const L = {
  map: jest.fn(() => mapObj),
  tileLayer: jest.fn(() => makeLayer()),
  marker: jest.fn(() =>
    makeLayer({
      setLatLng: jest.fn(ret),
      bindPopup: jest.fn(ret),
      setIcon: jest.fn(ret),
    })
  ),
  polyline: jest.fn(() =>
    makeLayer({
      setLatLngs: jest.fn(noop),
    })
  ),
  circle: jest.fn(() => makeLayer()),
  circleMarker: jest.fn(() => makeLayer()),
  layerGroup: jest.fn(() =>
    makeLayer({
      addLayer: jest.fn(ret),
      clearLayers: jest.fn(noop),
    })
  ),
  featureGroup: jest.fn(() =>
    makeLayer({
      addLayer: jest.fn(ret),
    })
  ),
  control: {
    layers: jest.fn(() => makeLayer()),
    scale:  jest.fn(() => makeLayer()),
    zoom:   jest.fn(() => makeLayer()),
  },
  icon: jest.fn(() => ({})),
  divIcon: jest.fn(() => ({})),
  popup: jest.fn(() =>
    makeLayer({
      setContent: jest.fn(ret),
      setLatLng:  jest.fn(ret),
    })
  ),
  DomUtil: {
    create: jest.fn(() => document.createElement('div')),
    addClass: jest.fn(noop),
    removeClass: jest.fn(noop),
    setTransform: jest.fn(noop),
  },
  CRS: { EPSG3857: {} },
  latLng: jest.fn((lat, lng) => ({ lat, lng })),
  latLngBounds: jest.fn(() => ({})),
  point: jest.fn((x, y) => ({ x, y })),
  Icon: { Default: function () {} },
};

module.exports = L;
