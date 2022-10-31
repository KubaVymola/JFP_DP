import "@babylonjs/core/Debug/debugLayer";
import "@babylonjs/inspector";
import "@babylonjs/loaders/glTF";
import {
    Engine,
    Scene,
    Vector3,
    HemisphericLight,
    Mesh,
    SceneLoader,
    StandardMaterial,
    Color3,
    AxesViewer,
    CreatePlane,
} from "@babylonjs/core";
import { OBJFileLoader } from "babylonjs-loaders";

import OriginCamera from "./originCamera";
import Entity from "./entity";
import Craft from "./craft";
import {
    createCamera,
    createEarth,
    createEntity,
} from './utils'
import {
    EARTH_RADIUS,
    SIM_DATA_ORIGIN,
} from './constants';

class App {
    ws: WebSocket;
    wsConnectTimeoutId: number = null;
    
    constructor() {
        const canvas: HTMLCanvasElement = document.querySelector('#gameCanvas');

        const loader: OBJFileLoader = new OBJFileLoader();
        SceneLoader.Append('/', '')

        document.body.appendChild(canvas);

        /**
         * initialize babylon scene and engine
         */
        const engine: Engine = new Engine(canvas, true);
        const scene: Scene = new Scene(engine);
        scene.clearColor.set(0, 0, 0, 1);
        scene.useRightHandedSystem = true;

        const camera: OriginCamera = createCamera(scene, canvas);
        camera.upVector = new Vector3(0, 0, 1);

        /**
         * Create Earth
         */
        const earthEntity: Entity = createEntity('earthEntity', scene, camera);
        const earthSphere: Mesh = createEarth(scene, earthEntity);

        const light: HemisphericLight = new HemisphericLight("light1", new Vector3(1, 0, 0), scene);

        const axes = new AxesViewer(scene, 0.25);
        const axesEntity = createEntity('axesEntity', scene, camera);
        axesEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        axes.xAxis.parent = axesEntity;
        axes.yAxis.parent = axesEntity;
        axes.zAxis.parent = axesEntity;

        const groundPlane = CreatePlane('groundPlane', { size: 200 });
        groundPlane.position = new Vector3(0, 0, 0);
        groundPlane.rotation = new Vector3(Math.PI, 0, 0);
        const groundPlaneEntity = createEntity('groundPlaneEntity', scene, camera);
        groundPlaneEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        groundPlane.parent = groundPlaneEntity;
        var groundMaterial = new StandardMaterial('groundMaterial', scene);
        groundMaterial.diffuseColor = new Color3(65 / 255, 152 / 255, 10 / 255);
        groundPlane.material = groundMaterial;
        
        /**
         * Hide/show the Inspector
         */
        window.addEventListener("keydown", (ev) => {
            if (ev.shiftKey && ev.ctrlKey && ev.key === 'I') {
                if (scene.debugLayer.isVisible()) {
                    scene.debugLayer.hide();
                } else {
                    scene.debugLayer.show();
                }
            }
        });

        /**
         * Connect to websocket
         */
        const urlInput: HTMLInputElement = document.querySelector('#connectInput');
        urlInput.value = SIM_DATA_ORIGIN;
        
        const button: HTMLButtonElement = document.querySelector('#connectButton');
        button.onclick = () => {
            connect(false);
        }

        /**
         * Create craft
         */
        const craft = new Craft(scene, camera);

        const connect = (reconnect: boolean) => {
            if (reconnect || !this.ws) {
                this.ws = new WebSocket(urlInput.value);

                button.innerText = 'Connecting...';
                
                this.ws.onopen = () => {
                    console.log('Connected');

                    button.innerText = 'Disconnect';

                    this.ws.onmessage = (event) => {
                        const parsedData = JSON.parse(event.data);
                        craft.update(parsedData, camera);
                    };
                };
                
                this.ws.onclose = () => {
                    this.wsConnectTimeoutId = setTimeout(() => {
                        console.log('Close: Trying to reconnect');
                        connect(true);
                    }, 500);
                }

                
            } else {
                if (this.wsConnectTimeoutId) clearTimeout(this.wsConnectTimeoutId);
                this.ws.onclose = null;
                this.ws.close();
                this.ws = null;

                button.innerText = 'Connect';
            }
        }

        /**
         * Start running
         */
        canvas.focus();
        
        engine.runRenderLoop(() => {
            scene.render();
        });
    }
}

new App();

export default App;
