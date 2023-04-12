/* eslint-disable @typescript-eslint/no-explicit-any */
/* eslint-disable no-case-declarations */
import {
    SceneLoader,
    Vector3,
    Matrix,
    Quaternion,
    Scene,
    TransformNode,
    CreateCylinder,
    Mesh,
    StandardMaterial,
    Color3,
    AbstractMesh,
} from '@babylonjs/core';

import Entity from './entity';
import {
    createEntity,
} from './utils';
import {
    EARTH_RADIUS,
} from './constants';
import OriginCamera from './originCamera';
import CraftEntity from './craftEntity';

declare global {
    interface Window {
        api: any,
    }
}

interface NodeTreeItem {
    mesh?: string,
    position?: Vector3,
    pivot?: Vector3,
    axis?: Vector3,
    angleProperty?: string,
    angleOffset?: number,
    spinnerProperty?: string,
    propSenseProperty?: string,
    thrustProperty?: string,
    enginePitchProperty?: string,
    engineYawProperty?: string,
    children?: Array<NodeTreeItem>,
}

class Craft {
    rootEntity: Entity = null;
    craftEntities: Array<CraftEntity> = [];
    nodeTree: object = {};
    lastTime: number;
    gamepadId: number = null;
    toJSBSimNames: Array<string> = [];
    
    constructor(scene: Scene, camera: OriginCamera) {
        this.rootEntity = createEntity('craftEntity', scene, camera);
        
        /**
         * TODO provide settings for the following
         * ! The craft, after loading, should have the nose to +x, up to -z and left -y
         * To achieve this, do the following in blender:
         * 1. The craft has fwd pointing to -x, left is to -y, and bottom to -z
         * 2. Export the craft with the default blender export settings (+y forward, -z up)
         */

        this.rootEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        // this.rootEntity.setPivotPoint(new Vector3(0, 0, 0));

        const inputButton: HTMLButtonElement = document.querySelector('#open-root-dir');
        inputButton.addEventListener('click', () => this.handleLoadAllData(scene));

        window.addEventListener('gamepadconnected', (e: GamepadEvent) => {
            this.gamepadId = e.gamepad.index;
        });

        window.addEventListener('gamepaddisconnected', () => {
            this.gamepadId = null;
        });
    }

    update(craftData: any, camera: OriginCamera) {
        /**
         * [?] - my coordinates
         * {?} - JSBSim coordinates
         * 
         * Update crafts position and rotation
         * [X] = {Y}   // East
         * [Y] = {Z}   // North
         * [Z] = {X}   // Up
         * s = latitude
         * t = longitude
         */

        // console.log(craftData);

        const deltaT_s = Math.max((craftData["simulation/sim-time-sec"] || 0) - this.lastTime, 0);
        this.lastTime = craftData["simulation/sim-time-sec"] || 0;
        
        // latitude -> North-south
        // longitude -> East-west
        const latitude_rad = craftData['ext/latitude-rad'];
        const longitude_rad = craftData['ext/longitude-rad'];

        const craftRadius = EARTH_RADIUS + craftData['ext/altitude-m'];
        
        /**
         * Source coordinates: JSBSim
         * Target coordinates: viz
         */
        const craftCG = new Vector3(
            -craftData['ext/cg-x-m'],
             craftData['ext/cg-y-m'],
            -craftData['ext/cg-z-m'],
        );
        
        const rocketPos = new Vector3(
            craftRadius * Math.sin(longitude_rad) * Math.cos(latitude_rad),   // east
            craftRadius * Math.sin(latitude_rad),                             // north
            craftRadius * Math.cos(longitude_rad) * Math.cos(latitude_rad),   // up
        )
        
        this.rootEntity.doublepos = rocketPos.subtract(craftCG);
        this.rootEntity.setPivotPoint(craftCG);
        camera.doubletgt = rocketPos;

        camera.upVector = rocketPos.normalize();
        
        /**
         * JSBSIM uses (w, i, j, k), babylon.js uses (i, j, k, w),
         * where w is the real part of quaternion. Hence the order of (2, 3, 4, 1)
         */
        const rotationQuaternion: Quaternion = new Quaternion(
            craftData['ext/ecef-q-2'],
            craftData['ext/ecef-q-3'],
            craftData['ext/ecef-q-4'],
            craftData['ext/ecef-q-1'],
        );

        let rotationMatrix: Matrix = Matrix.Identity();

        const quaternionMatrix: Matrix = Matrix.Zero()
        rotationQuaternion.toRotationMatrix(quaternionMatrix);
        
        const fromJSBSimCoords: Matrix = Matrix.Zero();
        fromJSBSimCoords.setRowFromFloats(0, 0, 0, 1, 0);
        fromJSBSimCoords.setRowFromFloats(1, 1, 0, 0, 0);
        fromJSBSimCoords.setRowFromFloats(2, 0, 1, 0, 0);
        fromJSBSimCoords.setRowFromFloats(3, 0, 0, 0, 1);

        /**
         * JSBSim rotates the craft -90° around y-axis in 0-yaw 0-pich state
         * Therefore it must be ensured that once this rotation is applied,
         * the craft will be pointing to the north with the long and cross axes
         * parallel to the ground.
         * The following two matrix multiplications ensure that the craft is pointing
         * to the north.
         */
        // rotationMatrix = rotationMatrix.multiply(Matrix.RotationAxis(new Vector3(0, 1, 0), Math.PI));
        // rotationMatrix = rotationMatrix.multiply(Matrix.RotationAxis(new Vector3(1, 0, 0), -Math.PI/2));
        /**
         * This rotation matrix is obtained from a quaternion directly from JSBSim's
         * ECEF frame. It is right handed Up-East-North frame
         */
        rotationMatrix = rotationMatrix.multiply(quaternionMatrix);
        /**
         * Transform the UEN frame to a frame that I use, which is ENU
         */
        rotationMatrix = rotationMatrix.multiply(fromJSBSimCoords);
        
        this.rootEntity.rotationQuaternion = Quaternion.FromRotationMatrix(rotationMatrix);

        for (const childEntity of this.craftEntities) {
            childEntity.updateFromSim(craftData, deltaT_s);
        }
    }

    async handleLoadAllData(scene: Scene) {
        /**
         * Let user select the root definition file
         */
        const { path, directory } = await window.api.openDialog({
            filters: [
                { name: 'JSBSim definition', extensions: ['xml'], },
            ],
            properties: ['openFile']
        });
        console.log(path);
        
        
        
        if (path) {
            const buffer: string = await window.api.readFile(path, { encoding: 'utf-8' });    
            await this.parseJSBSimDef(buffer, directory, scene);
        }
    }

    async parseJSBSimDef(xmlText: string, rootDir: string, scene: Scene) {
        const objPaths: string[] = [];
        
        const parser = new DOMParser();
        const xmlDoc = parser.parseFromString(xmlText, 'application/xml');

        const wsEl = xmlDoc.documentElement.getElementsByTagName('ws');

        if (!wsEl[0]) return;

        const nodeEls = wsEl[0].children;

        for (let i = 0; i < nodeEls.length; ++i) {
            if (nodeEls[i].nodeName === 'to_jsbsim') {
                this.parseToJSBSimNode(nodeEls[i]);
            }
            
            if (nodeEls[i].nodeName === 'file') {
                objPaths.push(nodeEls[i].textContent);
            }
            
            if (nodeEls[i].nodeName === 'node') {
                this.nodeTree = this.parseWSNode(nodeEls[i]);
            }
        }


        const objFiles: File[] = [];
        
        for (const objPath of objPaths) {
            const fullPath = await window.api.joinPath(rootDir, objPath);
            console.log('loading obj file @', fullPath);
            
            const data: string = await window.api.readFile(fullPath, { encoding: 'utf-8' });
            
            objFiles.push(new File([data], fullPath));
        }

        console.log(this.nodeTree);

        this.loadMeshEntityFromTree(scene, this.nodeTree, this.rootEntity, objFiles);
    }

    parseToJSBSimNode(parentNode: Element) {
        const children = parentNode.children;
        
        for (let i = 0; i < children.length; ++i) {
            if (children[i].nodeName === 'property') {
                this.toJSBSimNames.push(children[i].textContent.trim());
            }
        }

        console.log(this.toJSBSimNames);
    }

    parseWSNode(currentNode: Element): object {
        const toReturn: NodeTreeItem = {
            mesh: null,
            position: new Vector3(0, 0, 0),
            pivot: new Vector3(0, 0, 0),
            axis: new Vector3(1, 0, 0),
            spinnerProperty: null,
            propSenseProperty: null,
            thrustProperty: null,
            enginePitchProperty: null,
            engineYawProperty: null,
            angleProperty: null,
            angleOffset: 0,
            children: [],
        };

        const mesh = currentNode.getAttribute('mesh');
        toReturn.mesh = mesh;
        
        const childEls = currentNode.children;

        for (let i = 0; i < childEls.length; ++i) {
            switch (childEls[i].nodeName) {
            case 'node':
                toReturn.children.push(this.parseWSNode(childEls[i]));
                break;
            case 'position':
                const positionEls = childEls[i].children;

                let posX, posY, posZ;
                for (let j = 0; j < positionEls.length; ++j) {
                    /**
                     * TODO maybe add position offset 
                     */
                    
                    if (positionEls[j].nodeName === 'x') posX = -Number(positionEls[j].textContent);
                    if (positionEls[j].nodeName === 'y') posY =  Number(positionEls[j].textContent);
                    if (positionEls[j].nodeName === 'z') posZ = -Number(positionEls[j].textContent);
                }

                toReturn.position = new Vector3(posX, posY, posZ);
                break;
            case 'pivot':
                const pivotEls = childEls[i].children;

                let pivotX = 0, pivotY = 0, pivotZ = 0;
                for (let j = 0; j < pivotEls.length; ++j) {
                    if (pivotEls[j].nodeName === 'x') pivotX = -Number(pivotEls[j].textContent);
                    if (pivotEls[j].nodeName === 'y') pivotY =  Number(pivotEls[j].textContent);
                    if (pivotEls[j].nodeName === 'z') pivotZ = -Number(pivotEls[j].textContent);
                }

                toReturn.pivot = new Vector3(pivotX, pivotY, pivotZ);
                break;
            case 'axis':
                const axisEls = childEls[i].children;

                let axisX = 0, axisY = 0, axisZ = 0;
                for (let j = 0; j < axisEls.length; ++j) {
                    if (axisEls[j].nodeName === 'x') axisX = -Number(axisEls[j].textContent);
                    if (axisEls[j].nodeName === 'y') axisY =  Number(axisEls[j].textContent);
                    if (axisEls[j].nodeName === 'z') axisZ = -Number(axisEls[j].textContent);
                }

                toReturn.axis = new Vector3(axisX, axisY, axisZ);
                break;
            case 'spinner_property':
                toReturn.spinnerProperty = childEls[i].textContent.trim();
                break;
            case 'prop_sense_property':
                toReturn.propSenseProperty = childEls[i].textContent.trim();
                break;
            case 'thrust_property':
                toReturn.thrustProperty = childEls[i].textContent.trim();
                break;
            case 'engine_pitch_property':
                toReturn.enginePitchProperty = childEls[i].textContent.trim();
                break;
            case 'engine_yaw_property':
                toReturn.engineYawProperty = childEls[i].textContent.trim();
                break;
            case 'angle_property':
                toReturn.angleProperty = childEls[i].textContent.trim();
                break;
            case 'angle_offset':
                toReturn.angleOffset = Number(childEls[i].textContent);
            break;
            }
        }

        return toReturn;
    }

    async loadMeshEntityFromTree(scene: Scene,
                                 currentDefNode: NodeTreeItem,
                                 parentEntity: TransformNode,
                                 objFiles: File[]) {
        
        if (!objFiles) return;
        if (!currentDefNode) return;
        
        console.log(currentDefNode.mesh || 'Loading node without mesh');

        const newCraftEntity = new CraftEntity('', scene);
        newCraftEntity.parent = parentEntity;
        // newCraftEntity.setposi = currentDefNode.position;
        newCraftEntity.position = currentDefNode.position;
        newCraftEntity.setPivotPoint(currentDefNode.pivot);
        newCraftEntity.axis = currentDefNode.axis;
        newCraftEntity.angleProperty = currentDefNode.angleProperty;
        newCraftEntity.angleOffset = currentDefNode.angleOffset;
        newCraftEntity.spinnerProperty = currentDefNode.spinnerProperty;
        newCraftEntity.propSenseProperty = currentDefNode.propSenseProperty;
        newCraftEntity.thrustProperty = currentDefNode.thrustProperty;
        newCraftEntity.enginePitchProperty = currentDefNode.enginePitchProperty;
        newCraftEntity.engineYawProperty = currentDefNode.engineYawProperty;
        this.craftEntities.push(newCraftEntity);

        if (currentDefNode.mesh) {
            const mesh: AbstractMesh = await this.loadMeshFromFiles(objFiles, currentDefNode.mesh, scene);
            if (mesh) mesh.parent = newCraftEntity;
        }

        if (currentDefNode.thrustProperty) {
            const height = 0.3;
            
            const mesh: Mesh = CreateCylinder(currentDefNode.thrustProperty, {
                height,
                diameterBottom: 0.15,
                diameterTop: 0,
                sideOrientation: Mesh.FRONTSIDE,
            }, scene);

            const thrustMaterial: StandardMaterial = new StandardMaterial(currentDefNode.thrustProperty + 'Mat', scene);
            thrustMaterial.diffuseColor = new Color3(1, 0, 0);
            thrustMaterial.alpha = 0.3;

            mesh.material = thrustMaterial;
            mesh.rotation = new Vector3(0, 0, -Math.PI / 2);
            mesh.position = new Vector3(-height / 2, 0, 0);
            // newCraftEntity.setPivotPoint(currentDefNode.position);
            mesh.parent = newCraftEntity;
        }

        
        if (currentDefNode.children) {
            for (const child of currentDefNode.children) {
                await this.loadMeshEntityFromTree(scene, child, newCraftEntity, objFiles);
            }
        }
        
    }


    async loadMeshFromFiles(objFiles: File[], meshName: string, scene: Scene): Promise<AbstractMesh> {
        for (let i = 0; i < objFiles.length; ++i) {
            const meshes = (await SceneLoader.ImportMeshAsync(meshName, '', objFiles[i], scene)).meshes;
        
            console.log('meshes len', meshes.length);

            if (meshes.length == 0) continue;
            return Promise.resolve(meshes[0]);
        }

        return null;
    }

    getGamepadData() {
        // let gamepad: Gamepad = null;

        // TODO use ternary operator

        const gamepad = navigator.getGamepads().length > 0
            ? navigator.getGamepads()[0]
            : null;

        // console.log('gamepad count', navigator.getGamepads().length);

        return this.toJSBSimNames.reduce((acc, name, id) => {
            if (gamepad === null
            || !gamepad.connected
            || id >= gamepad.axes.length) return { ...acc, [name]: 0.0 };
            
            return { ...acc, [name]: gamepad.axes[id] };
        }, {});

        
        // if (navigator.getGamepads().length > 0) {
        //     gamepad = navigator.getGamepads()[0];
        // }
        
        // const toReturn: any = {};

        // if (gamepad === null || !gamepad.connected) {
        //     this.toJSBSimNames.forEach((name) => {
        //         toReturn[name] = 0.0;
        //     });
            
        //     return toReturn;
        // }

        // // TODO reduce this to single reduce
        
        // this.toJSBSimNames.forEach((name, id) => {
        //     if (id >= gamepad.axes.length) toReturn[name] = 0.0;
        //     else toReturn[name] = gamepad.axes[id];
        // });

        // return toReturn;
    }
}

export default Craft;
