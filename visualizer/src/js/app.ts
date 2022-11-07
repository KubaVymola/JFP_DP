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
    connectButton: HTMLButtonElement;
    urlInput: HTMLInputElement;
    
    camera: OriginCamera;
    craft: Craft;

    engine: Engine;
    scene: Scene;
    
    constructor() {
        const canvas: HTMLCanvasElement = document.querySelector('#gameCanvas');

        const loader: OBJFileLoader = new OBJFileLoader();
        SceneLoader.Append('/', '')

        document.body.appendChild(canvas);

        /**
         * initialize babylon scene and engine
         */
        this.engine = new Engine(canvas, true);
        this.scene = new Scene(this.engine);
        this.scene.clearColor.set(0, 0, 0, 1);
        this.scene.useRightHandedSystem = true;

        this.camera = createCamera(this.scene, canvas);
        this.camera.upVector = new Vector3(0, 0, 1);

        /**
         * Create Earth
         */
        const earthEntity: Entity = createEntity('earthEntity', this.scene, this.camera);
        const earthSphere: Mesh = createEarth(this.scene, earthEntity);

        const light: HemisphericLight = new HemisphericLight("light1", new Vector3(1, 0, 0), this.scene);

        /**
         * Create origin axes
         */
        const axes = new AxesViewer(this.scene, 0.25);
        const axesEntity = createEntity('axesEntity', this.scene, this.camera);
        axesEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        axes.xAxis.parent = axesEntity;
        axes.yAxis.parent = axesEntity;
        axes.zAxis.parent = axesEntity;

        /**
         * Create basic ground
         */
        const groundPlane = CreatePlane('groundPlane', { size: 200 });
        groundPlane.position = new Vector3(0, 0, 0);
        groundPlane.rotation = new Vector3(Math.PI, 0, 0);
        const groundPlaneEntity = createEntity('groundPlaneEntity', this.scene, this.camera);
        groundPlaneEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        groundPlane.parent = groundPlaneEntity;
        var groundMaterial = new StandardMaterial('groundMaterial', this.scene);
        groundMaterial.diffuseColor = new Color3(65 / 255, 152 / 255, 10 / 255);
        groundPlane.material = groundMaterial;
        
        /**
         * Hide/show the Inspector
         */
        window.addEventListener("keydown", (ev) => {
            if (ev.shiftKey && ev.ctrlKey && ev.key === 'I') {
                if (this.scene.debugLayer.isVisible()) {
                    this.scene.debugLayer.hide();
                } else {
                    this.scene.debugLayer.show();
                }
            }
        });

        /**
         * Connect to websocket
         */
        this.urlInput = document.querySelector('#connectInput');
        this.urlInput.value = SIM_DATA_ORIGIN;
        
        this.connectButton = document.querySelector('#connectButton');
        this.connectButton.onclick = () => {
            this.connect(false);
        }

        /**
         * Create craft
         */
        this.craft = new Craft(this.scene, this.camera);

        /**
         * Start running
         */
        canvas.focus();
        
        this.engine.runRenderLoop(() => {
            this.scene.render();
        });
    }

    connect(reconnect: boolean) {
        if (reconnect || !this.ws) {
            this.ws = new WebSocket(this.urlInput.value);

            this.connectButton.innerText = 'Connecting...';
            
            this.ws.onopen = () => {
                console.log('Connected');

                this.connectButton.innerText = 'Disconnect';

                this.ws.onmessage = (event) => {
                    const parsedData = JSON.parse(event.data);
                    this.craft.update(parsedData, this.engine.getDeltaTime(), this.camera);
                };
            };
            
            this.ws.onclose = () => {
                this.wsConnectTimeoutId = setTimeout(() => {
                    console.log('Close: Trying to reconnect');
                    this.connect(true);
                }, 500);
            }

            
        } else {
            if (this.wsConnectTimeoutId) clearTimeout(this.wsConnectTimeoutId);
            this.ws.onclose = null;
            this.ws.close();
            this.ws = null;

            this.connectButton.innerText = 'Connect';
        }
    }
}

new App();

export default App;
