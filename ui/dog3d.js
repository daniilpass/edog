// 3D-модель собаки. Детали берутся из /models/manifest.json (cad/web/),
// если манифеста нет — собака рисуется примитивами по геометрии из /api/config.
//
// Оси сцены: x — вперёд, y — вверх, z — вправо (мм).
// Иерархия: root → body → hipBase[leg] → hip[leg] (вращается) → knee[leg] (вращается).
// Без манифеста hipBase правых ног отражён по z, и примитивы описываются один раз — для левой стороны.
// Манифест задаёт оси бёдер (hips) и детали для каждой ноги отдельно — тогда отражения нет.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { STLLoader } from './vendor/STLLoader.js';

const MG90S = { w: 22.8, d: 12.2, h: 22.5 }; // корпус без ушей; вал смещён к краю
const COLORS = { body: 0xe8e2d6, left: 0x5b92f0, right: 0xf08a3c, servo: 0x2b4fa8, joint: 0xffffff };

export class Dog3D {
  constructor(container, config, { onPick } = {}) {
    this.config = config;
    this.onPick = onPick;
    this.legs = {};
    this.markers = [];
    this.selected = null;

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 1, 5000);
    this.camera.position.set(260, 220, 320);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 60, 0);
    this.controls.enableDamping = true;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x667788, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(200, 400, 150);
    this.scene.add(sun);

    const grid = new THREE.GridHelper(600, 30, 0x888888, 0x888888);
    grid.material.opacity = 0.25;
    grid.material.transparent = true;
    this.scene.add(grid);
    // Перёд собаки (+X): корпус почти симметричен, без метки легко перепутать стороны.
    this.scene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(130, 0.5, 0), 70, 0x3cc47a, 22, 14));

    this.root = new THREE.Group();
    this.body = new THREE.Group();
    this.root.add(this.body);
    this.scene.add(this.root);
    this._buildSkeleton();

    this.raycaster = new THREE.Raycaster();
    this._pointerDown = null;
    const el = this.renderer.domElement;
    el.addEventListener('pointerdown', e => { this._pointerDown = [e.clientX, e.clientY]; });
    el.addEventListener('pointerup', e => this._pick(e));

    new ResizeObserver(() => this._resize(container)).observe(container);
    this._resize(container);
    this.renderer.setAnimationLoop(() => {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  _buildSkeleton() {
    const g = this.config.geometry;
    for (const leg of this.config.legs) {
      const right = leg.side === 'right';
      const hipBase = new THREE.Group();
      hipBase.position.set(leg.x_mm, 0, (right ? 1 : -1) * g.body_width_mm / 2);
      if (right) hipBase.scale.z = -1;
      const hip = new THREE.Group();
      const knee = new THREE.Group();
      knee.position.y = -g.femur_mm;
      const foot = new THREE.Object3D();
      foot.position.y = -g.tibia_mm;
      knee.add(foot);
      hip.add(knee);
      hipBase.add(hip);
      this.body.add(hipBase);
      this.legs[leg.name] = { cfg: leg, hipBase, hip, knee, foot, color: right ? COLORS.right : COLORS.left };

      for (const [joint, parent] of [['hip', hipBase], ['knee', hip]]) {
        const m = new THREE.Mesh(
          new THREE.CylinderGeometry(5, 5, 4, 24).rotateX(Math.PI / 2),
          new THREE.MeshStandardMaterial({ color: COLORS.joint, emissive: 0x000000 }));
        m.position.set(0, joint === 'knee' ? -g.femur_mm : 0, -9);
        m.userData.servo = `${leg.name}_${joint}`;
        m.userData.leg = leg.name;
        parent.add(m);
        this.markers.push(m);
      }
    }
  }

  // ---------- детали ----------

  async loadModels(url = '/models/manifest.json') {
    let manifest = null;
    try {
      const r = await fetch(url);
      if (r.ok) manifest = await r.json();
    } catch { /* нет манифеста */ }
    if (!manifest?.parts?.length) {
      this._buildPrimitives();
      return 'примитивы';
    }
    this._applyManifestGeometry(manifest);
    const loader = new STLLoader();
    const base = url.slice(0, url.lastIndexOf('/') + 1);
    const cache = {};
    let loaded = 0;
    for (const part of manifest.parts) {
      try {
        cache[part.file] ??= loader.loadAsync(base + part.file).then(geo => { geo.computeVertexNormals(); return geo; });
        const geo = await cache[part.file];
        for (const parent of this._attachTargets(part)) {
          const mat = new THREE.MeshStandardMaterial({
            color: part.color ? new THREE.Color(part.color) : this._legColor(parent),
            roughness: 0.75, metalness: 0.05,
          });
          const mesh = new THREE.Mesh(geo, mat);
          applyTransform(mesh, part);
          parent.add(mesh);
        }
        loaded++;
      } catch (e) {
        console.warn('деталь не загрузилась', part.file, e);
      }
    }
    if (!loaded) this._buildPrimitives();
    return loaded ? `детали: ${loaded}/${manifest.parts.length}` : 'примитивы';
  }

  _applyManifestGeometry(manifest) {
    const g = manifest.geometry ?? {};
    for (const [name, leg] of Object.entries(this.legs)) {
      const hip = manifest.hips?.[name];
      if (hip) {
        leg.hipBase.position.set(...hip);
        leg.hipBase.scale.z = 1;
      }
      if (g.femur_mm) leg.knee.position.y = -g.femur_mm;
      if (g.tibia_mm) leg.foot.position.y = -g.tibia_mm;
      // Подошва внутри от оси бедра (голень стоит внутри бедра) — важно для крена.
      if (g.foot_inset_mm) leg.foot.position.z = (leg.cfg.side === 'left' ? 1 : -1) * g.foot_inset_mm;
      // Маркер сустава — снаружи бедра: у левых ног наружу −z, у правых (без отражения) +z.
      const outward = leg.hipBase.scale.z < 0 || leg.cfg.side === 'left' ? -1 : 1;
      for (const m of this.markers) {
        if (m.userData.leg !== name) continue;
        m.position.z = outward * 12;
        if (m.userData.servo.endsWith('knee')) m.position.y = leg.knee.position.y;
      }
    }
  }

  _attachTargets(part) {
    if (part.attach === 'body') return [this.body];
    const names = part.legs ?? Object.keys(this.legs);
    return names.filter(n => this.legs[n]).map(n => this.legs[n][{ hip_base: 'hipBase', hip: 'hip', knee: 'knee' }[part.attach]]);
  }

  _legColor(obj) {
    for (const leg of Object.values(this.legs)) {
      if (obj === leg.hipBase || obj === leg.hip || obj === leg.knee) return leg.color;
    }
    return COLORS.body;
  }

  _buildPrimitives() {
    const g = this.config.geometry;
    const mat = c => new THREE.MeshStandardMaterial({ color: c, roughness: 0.8 });
    const box = (w, h, d, c) => new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat(c));

    const chassis = box(g.body_length_mm, 30, g.body_width_mm - 6, COLORS.body);
    chassis.position.y = 12;
    this.body.add(chassis);
    const head = box(18, 22, 40, COLORS.body);
    head.position.set(g.body_length_mm / 2 + 6, 16, 0);
    this.body.add(head);

    for (const leg of Object.values(this.legs)) {
      // Серво бедра — в корпусе, вал на оси бедра.
      const hs = box(MG90S.w, MG90S.h, MG90S.d, COLORS.servo);
      hs.position.set(-MG90S.w / 2 + 6, 0, 1);
      leg.hipBase.add(hs);
      // Бедро.
      const femur = box(12, g.femur_mm + 12, 5, leg.color);
      femur.position.set(0, -g.femur_mm / 2, -7);
      leg.hip.add(femur);
      // Серво колена — в голени, вал на оси колена.
      const ks = box(MG90S.d, MG90S.w, MG90S.h, COLORS.servo);
      ks.position.set(0, -MG90S.w / 2 + 6, 6);
      leg.knee.add(ks);
      // Голень.
      const tibia = box(9, g.tibia_mm, 8, leg.color);
      tibia.position.set(0, -g.tibia_mm / 2, 0);
      leg.knee.add(tibia);
      const paw = new THREE.Mesh(new THREE.SphereGeometry(6, 16, 12), mat(0x333333));
      leg.foot.add(paw);
    }
  }

  // ---------- состояние ----------

  update(state) {
    const byId = Object.fromEntries(state.servos.map(s => [s.id, s]));
    for (const [name, leg] of Object.entries(this.legs)) {
      const hip = byId[`${name}_hip`], knee = byId[`${name}_knee`];
      if (hip) leg.hip.rotation.z = THREE.MathUtils.degToRad(hip.joint_current);
      // Угол колена — по корпусу голени; модель голени повёрнута по линии «колено → подошва».
      if (knee) leg.knee.rotation.z = THREE.MathUtils.degToRad(knee.joint_current - (this.config.geometry.tibia_bend_deg ?? 0));
    }
    for (const m of this.markers) {
      const s = byId[m.userData.servo];
      const c = m.material;
      c.color.set(m.userData.servo === this.selected ? 0xffd23c
        : s?.clamped ? 0xf05b5b : s?.installed ? 0x3cc47a : 0x777777);
      c.emissive.set(m.userData.servo === this.selected ? 0x554400 : 0x000000);
    }

    if (this.staticBody) this._hangOnStand(); else this._settleOnFloor();
    this.renderer.domElement.style.opacity = state.enabled ? 1 : 0.75;
  }

  // Режим стенда: корпус неподвижно висит ровно, лапы не достают до пола — видно только работу ног.
  _hangOnStand() {
    const g = this.config.geometry;
    this.body.quaternion.identity();
    this.root.position.set(0, g.femur_mm + g.tibia_mm + 25, 0);
  }

  setStaticBody(on) { this.staticBody = on; }

  // Положение корпуса не берётся из команды, а вычисляется по углам суставов:
  // лапы (FK) в системе корпуса → плоскость опорных лап → корпус поворачивается так,
  // чтобы эта плоскость стала полом. Лапы, поднятые над плоскостью (перенос), в опору не идут.
  _settleOnFloor() {
    this.body.quaternion.identity();
    this.root.position.set(0, 0, 0);
    this.root.updateMatrixWorld(true);
    const feet = Object.values(this.legs).map(l => l.foot.getWorldPosition(new THREE.Vector3()));

    let w = feet.map(() => 1);
    let plane = fitPlane(feet, w);
    for (let i = 0; i < 2 && plane; i++) {
      w = feet.map(f => (f.y - (plane.a * f.x + plane.b * f.z + plane.c) > 2 ? 0.02 : 1));
      plane = fitPlane(feet, w) ?? plane;
    }
    if (plane) {
      const normal = new THREE.Vector3(-plane.a, 1, -plane.b).normalize();
      this.body.quaternion.setFromUnitVectors(normal, new THREE.Vector3(0, 1, 0));
    }
    this.root.updateMatrixWorld(true);
    let minY = Infinity;
    const p = new THREE.Vector3();
    for (const leg of Object.values(this.legs)) minY = Math.min(minY, leg.foot.getWorldPosition(p).y);
    this.root.position.y = -minY;
  }

  select(id) { this.selected = id; }

  // Плоскость рамы (низ средней перекладины, как FramePlane в FreeCAD). Узел колена держится
  // ниже неё на запас из прошивки. Привязана к корпусу; уровень — мм от оси бедра (y = 0).
  showFramePlane(visible) {
    const info = this.config.frame_guard;
    if (!info) return false;
    if (!this.framePlane) {
      const g = this.config.geometry;
      const w = g.body_length_mm + 140, d = g.body_width_mm + 80;
      const plane = new THREE.Mesh(
        new THREE.PlaneGeometry(w, d).rotateX(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: 0xf05b5b, transparent: true, opacity: 0.18,
          side: THREE.DoubleSide, depthWrite: false }));
      const edge = new THREE.LineSegments(
        new THREE.EdgesGeometry(plane.geometry),
        new THREE.LineDashedMaterial({ color: 0xd63b3b, dashSize: 6, gapSize: 4 }));
      edge.computeLineDistances();
      plane.add(edge);
      plane.position.y = info.plane_mm;   // как FramePlane в FreeCAD: низ перекладины рамы
      plane.renderOrder = 10;
      this.body.add(plane);
      this.framePlane = plane;
    }
    this.framePlane.visible = visible;
    return true;
  }

  _pick(e) {
    if (!this._pointerDown) return;
    const [x0, y0] = this._pointerDown;
    this._pointerDown = null;
    if (Math.hypot(e.clientX - x0, e.clientY - y0) > 5) return; // это было вращение камеры
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    const hit = this.raycaster.intersectObjects(this.markers)[0];
    if (hit) this.onPick?.(hit.object.userData.servo);
  }

  _resize(container) {
    const { clientWidth: w, clientHeight: h } = container;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
}

// Взвешенный МНК: плоскость y = a·x + b·z + c через точки.
function fitPlane(pts, w) {
  let sxx = 0, sxz = 0, szz = 0, sx = 0, sz = 0, s = 0, sxy = 0, szy = 0, sy = 0;
  pts.forEach((p, i) => {
    const k = w[i];
    sxx += k * p.x * p.x; sxz += k * p.x * p.z; szz += k * p.z * p.z;
    sx += k * p.x; sz += k * p.z; s += k;
    sxy += k * p.x * p.y; szy += k * p.z * p.y; sy += k * p.y;
  });
  const m = new THREE.Matrix3().set(sxx, sxz, sx, sxz, szz, sz, sx, sz, s);
  if (Math.abs(m.determinant()) < 1e-6) return null;
  const r = new THREE.Vector3(sxy, szy, sy).applyMatrix3(m.invert());
  return { a: r.x, b: r.y, c: r.z };
}

function applyTransform(obj, part) {
  const [rx = 0, ry = 0, rz = 0] = part.rotation ?? [];
  obj.rotation.set(...[rx, ry, rz].map(THREE.MathUtils.degToRad), part.rotation_order ?? 'XYZ');
  if (part.position) obj.position.set(...part.position);
  if (part.scale) obj.scale.set(...[].concat(part.scale, part.scale, part.scale).slice(0, 3));
}
