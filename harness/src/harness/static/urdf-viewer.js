import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import URDFLoader from './vendor/urdf-loader/src/URDFLoader.js';

const viewers = new WeakMap();
const versions = new WeakMap();
let modelPromise;

function loadRobot() {
  if (!modelPromise) modelPromise = new Promise((resolve, reject) => {
    let robot;
    const meshTasks = [];
    const manager = new THREE.LoadingManager();
    manager.onLoad = () => Promise.all(meshTasks).then(() => robot ? resolve(robot) : reject(new Error('URDF 모델을 읽지 못했습니다.')), reject);
    manager.onError = url => reject(new Error(`3D 메시를 읽지 못했습니다: ${url}`));
    const loader = new URDFLoader(manager), meshes = new Map();
    loader.loadMeshCb = (path, unusedManager, done) => {
      if (!meshes.has(path)) meshes.set(path, new STLLoader(manager).loadAsync(path));
      meshTasks.push(meshes.get(path).then(geometry => done(new THREE.Mesh(geometry, new THREE.MeshPhongMaterial()))));
    };
    loader.load('/models/so101_new_calib.urdf', value => { robot = value; }, undefined, reject);
  });
  return modelPromise;
}

function label(text, color = '#304238') {
  const canvas = document.createElement('canvas'); canvas.width = 256; canvas.height = 64;
  const context = canvas.getContext('2d');
  context.fillStyle = 'rgba(255,255,255,.94)'; context.fillRect(0, 0, 256, 64);
  context.fillStyle = color; context.font = 'bold 30px sans-serif'; context.textAlign = 'center'; context.textBaseline = 'middle';
  context.fillText(text, 128, 32);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(canvas), depthTest: false}));
  sprite.scale.set(.09, .0225, 1); sprite.renderOrder = 10;
  return sprite;
}

class Viewer {
  constructor(element, robot) {
    this.element = element;
    this.renderer = new THREE.WebGLRenderer({antialias: true, alpha: false});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0xf3f6f0);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.domElement.setAttribute('aria-label', 'SO-101 URDF 3D 로봇 팔. 마우스로 회전, 휠로 확대할 수 있습니다.');
    element.replaceChildren(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(38, 1, .001, 10);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.minDistance = .12; this.controls.maxDistance = 2.5;
    this.controls.addEventListener('change', () => this.render());
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x87927f, 2.4));
    const light = new THREE.DirectionalLight(0xffffff, 3); light.position.set(.4, -.4, .8); this.scene.add(light);
    const grid = new THREE.GridHelper(1, 20, 0xb3c2ae, 0xdce4d6); grid.rotation.x = Math.PI / 2; grid.position.z = -.004;
    this.scene.add(grid, new THREE.AxesHelper(.12));
    for (const [text, point, color] of [['X', [.14, 0, 0], '#b43f36'], ['Y', [0, .14, 0], '#438845'], ['Z', [0, 0, .14], '#3873aa']]) {
      const sprite = label(text, color); sprite.position.set(...point); sprite.scale.multiplyScalar(.55); this.scene.add(sprite);
    }
    this.current = robot.clone(true); this.target = robot.clone(true);
    for (const model of [this.current, this.target]) {
      // Display out-of-range feedback honestly; the controller separately rejects its IK.
      Object.values(model.joints).forEach(joint => { joint.ignoreLimits = true; });
      model.traverse(item => { if (item.isMesh) item.material = item.material.clone(); });
      this.scene.add(model);
    }
    this.target.traverse(item => { if (item.isMesh) { item.material.color.set(0x249a77); item.material.transparent = true; } });
    this.currentTip = new THREE.Mesh(new THREE.SphereGeometry(.003, 16, 12), new THREE.MeshBasicMaterial({color: 0x3975bd}));
    this.goal = new THREE.Mesh(new THREE.SphereGeometry(.004, 16, 12), new THREE.MeshBasicMaterial({color: 0xd17927}));
    this.arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), .01, 0xd17927, .004, .002);
    this.tipLabel = label('현재 팔 끝', '#3167a5');
    const baseLabel = label('팔 장착부'); baseLabel.position.set(-.03, 0, .02);
    this.scene.add(this.currentTip, this.goal, this.arrow, this.tipLabel, baseLabel);
    this.resize = new ResizeObserver(() => {
      const width = element.clientWidth;
      if (!width) return;
      const height = Math.max(300, Math.min(440, width * .88));
      this.renderer.setSize(width, height); this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix(); this.render();
    });
    this.resize.observe(element);
    this.view = null;
    element.dataset.loaded = 'true';
  }

  update(data, view, display) {
    const joints = data?.state?.fk?.joints_deg;
    this.current.visible = Boolean(joints);
    this.target.visible = Boolean(data?.result?.success && data.result.solution_urdf_joints_deg);
    this.currentTip.visible = this.current.visible;
    this.tipLabel.visible = this.current.visible;
    this.goal.visible = Boolean(data?.result?.target_xyz_m);
    this.arrow.visible = this.goal.visible && this.current.visible;
    if (joints) {
      this.current.setJointValues(Object.fromEntries(Object.entries(joints).map(([name, value]) => [name, THREE.MathUtils.degToRad(value)])));
      this.current.updateMatrixWorld(true);
      const tip = this.current.links.gripper_frame_link.getWorldPosition(new THREE.Vector3());
      this.element.dataset.tipXyz = JSON.stringify(tip.toArray());
      const expected = new THREE.Vector3(...data.state.current_xyz_m);
      if (tip.distanceTo(expected) > 1e-6) throw new Error('표시 모델과 IK 모델의 팔 끝 위치가 다릅니다. 이동 전 모델을 확인해주세요.');
      this.element.dataset.fkMatch = 'true';
      this.currentTip.position.copy(tip); this.tipLabel.position.copy(tip).add(new THREE.Vector3(0, 0, .022));
      if (this.goal.visible) {
        const target = new THREE.Vector3(...data.result.target_xyz_m), delta = target.clone().sub(tip), distance = delta.length();
        this.goal.position.copy(target); this.arrow.position.copy(tip);
        this.arrow.visible = distance > 1e-8;
        if (distance > 1e-8) { this.arrow.setDirection(delta.normalize()); this.arrow.setLength(distance, Math.min(.008, distance * .35), Math.min(.004, distance * .2)); }
      }
    }
    if (this.target.visible) {
      this.target.setJointValues(Object.fromEntries(Object.entries(data.result.solution_urdf_joints_deg).map(([name, value]) => [name, THREE.MathUtils.degToRad(value)])));
      this.target.updateMatrixWorld(true);
      const tip = this.target.links.gripper_frame_link.getWorldPosition(new THREE.Vector3());
      this.element.dataset.targetTipXyz = JSON.stringify(tip.toArray());
      if (tip.distanceTo(new THREE.Vector3(...data.result.achieved_xyz_m)) > 1e-6) throw new Error('목표 자세의 표시 위치와 IK 결과가 다릅니다.');
      this.target.traverse(item => { if (item.isMesh) { item.material.opacity = display === 'overlay' ? .32 : 1; item.material.depthWrite = display !== 'overlay'; } });
    }
    const hasTarget = this.target.visible;
    this.current.visible &&= display !== 'target' || !hasTarget;
    this.target.visible &&= display !== 'current';
    const poseKey = JSON.stringify(joints);
    if (this.view !== view || this.poseKey !== poseKey) {
      this.view = view; this.poseKey = poseKey;
      const bounds = new THREE.Box3().setFromObject(this.current);
      const center = joints ? bounds.getCenter(new THREE.Vector3()) : new THREE.Vector3(.13, 0, .16);
      const size = bounds.getSize(new THREE.Vector3());
      const distance = joints ? Math.max(.32, Math.max(size.x, size.y, size.z) * 2) : .82;
      const direction = {xz: [0, -1, .03], xy: [0, -.001, 1], yz: [1, 0, .03], iso: [1, -1.3, .8]}[view] || [1, -1.3, .8];
      this.camera.up.set(...(view === 'xy' ? [0, 1, 0] : [0, 0, 1]));
      this.camera.position.copy(center).add(new THREE.Vector3(...direction).normalize().multiplyScalar(distance));
      this.controls.target.copy(center); this.controls.update();
    }
    this.element.dataset.display = display;
    this.render();
  }

  render() { this.renderer.render(this.scene, this.camera); }
}

export async function updateViewer(element, data, view = 'iso', display = 'current') {
  const version = (versions.get(element) || 0) + 1; versions.set(element, version);
  if (!viewers.has(element)) {
    element.textContent = 'SO-101 URDF와 외형 메시를 불러오는 중…';
    viewers.set(element, loadRobot().then(robot => new Viewer(element, robot)));
  }
  const viewer = await viewers.get(element);
  if (versions.get(element) === version) viewer.update(data, view, display);
}
