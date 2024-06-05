import * as THREE from 'three';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import {
  setupGUI,
  downloadExampleScenesFolder,
  loadSceneFromURL,
  getPosition,
  getQuaternion,
  toMujocoPos,
  standardNormal,
} from './mujocoUtils.js';
import load_mujoco from '../dist/mujoco_wasm.js';

const mujoco = await load_mujoco();
const version = '1.0.0'; // You can automate this based on your build process

var initialScene = `humanoid.xml?v=${version}`;
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile(`/working/${initialScene}`, await (await fetch(`./examples/scenes/${initialScene}`)).text());

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    try {
      this.model = new mujoco.Model(`/working/${initialScene}`);
      this.state = new mujoco.State(this.model);
      this.simulation = new mujoco.Simulation(this.model, this.state);
    } catch (error) {
      console.error('Error loading model:', error);
    }

    this.params = {
      scene: initialScene,
      paused: false,
      help: false,
      ctrlnoiserate: 0.0,
      ctrlnoisestd: 0.0,
      keyframeNumber: 0
    };
    this.mujoco_time = 0.0;
    this.bodies = {};
    this.lights = {};
    this.tmpVec = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement('div');
    document.body.appendChild(this.container);

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 100);
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.1);
    this.ambientLight.name = 'AmbientLight';
    this.scene.add(this.ambientLight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.setAnimationLoop(this.render.bind(this));

    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', debounce(this.onWindowResize.bind(this), 250));

    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);
  }

  async init() {
    await downloadExampleScenesFolder(mujoco);

    try {
      [this.model, this.state, this.simulation, this.bodies, this.lights] = await loadSceneFromURL(mujoco, initialScene, this);
    } catch (error) {
      console.error('Error loading scene from URL:', error);
    }

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  render(timeMS) {
    this.controls.update();

    if (!this.params['paused']) {
      let timestep;
      try {
        timestep = this.model?.getOptions()?.timestep;
        if (timestep === undefined || timestep === null) {
          throw new Error('timestep is null or undefined');
        }
      } catch (error) {
        console.error('Error getting timestep from model options:', error);
        timestep = 0.016; // default timestep value in case of error
      }

      if (timeMS - this.mujoco_time > 35.0) {
        this.mujoco_time = timeMS;
      }

      while (this.mujoco_time < timeMS) {
        if (this.params['ctrlnoisestd'] > 0.0) {
          let rate = Math.exp(-timestep / Math.max(1e-10, this.params['ctrlnoiserate']));
          let scale = this.params['ctrlnoisestd'] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.simulation?.ctrl;
          if (currentCtrl) {
            for (let i = 0; i < currentCtrl.length; i++) {
              currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
              this.params['Actuator ' + i] = currentCtrl[i];
            }
          }
        }

        if (this.simulation?.qfrc_applied) {
          for (let i = 0; i < this.simulation.qfrc_applied.length; i++) {
            this.simulation.qfrc_applied[i] = 0.0;
          }
        }

        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition(this.simulation.xpos, b, this.bodies[b].position);
              getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }

          let bodyID = dragged.bodyID;
          this.dragStateManager.update();
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);
        }

        try {
          this.simulation?.step();
        } catch (error) {
          console.error('Error during simulation step:', error);
        }

        this.mujoco_time += timestep * 1000.0;
      }
    } else if (this.params['paused']) {
      this.dragStateManager.update();
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition(this.simulation.xpos, b, this.tmpVec, false); // Get raw coordinate from MuJoCo
        getQuaternion(this.simulation.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
        if (this.model.body_mocapid[b] >= 0) {
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos = this.simulation.mocap_pos;
          pos[addr + 0] += offset.x;
          pos[addr + 1] += offset.y;
          pos[addr + 2] += offset.z;
        }
      }

      this.simulation.forward();
    }

    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.simulation.xpos, b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    let numWraps = 0;
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let mat = new THREE.Matrix4();
      for (let t = 0; t < this.model.ntendon; t++) {
        let startW = this.simulation.ten_wrapadr[t];
        let r = this.model.tendon_width[t];
        for (let w = startW; w < startW + this.simulation.ten_wrapnum[t] - 1; w++) {
          let tendonStart = getPosition(this.simulation.wrap_xpos, w, new THREE.Vector3());
          let tendonEnd = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
          let tendonAvg = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

          let validStart = tendonStart.length() > 0.01;
          let validEnd = tendonEnd.length() > 0.01;

          if (validStart) {
            this.mujocoRoot.spheres.setMatrixAt(numWraps, mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r)));
          }
          if (validEnd) {
            this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd, new THREE.Quaternion(), new THREE.Vector3(r, r, r)));
          }
          if (validStart && validEnd) {
            mat.compose(tendonAvg, new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()), new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres.count = numWraps > 0 ? numWraps + 1 : 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres.instanceMatrix.needsUpdate = true;
    }

    this.renderer.render(this.scene, this.camera);
  }
}

let demo = new MuJoCoDemo();
await demo.init();

// Debounce function to prevent excessive calls
function debounce(func, wait) {
  let timeout;
  return function (...args) {
    const context = this;
    clearTimeout(timeout);
    timeout = setTimeout(() => func.apply(context, args), wait);
  };
}
