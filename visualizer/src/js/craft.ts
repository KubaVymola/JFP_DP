import {
    SceneLoader,
    Vector3,
    Matrix,
    Quaternion,
    Scene,
    TransformNode,
} from '@babylonjs/core';

import Entity from './entity';
import {
    createEntity,
} from './utils';
import {
    EARTH_RADIUS,
    DEG_TO_RAD,
} from './constants';
import OriginCamera from './originCamera';
import CraftEntity from './craftEntity';

interface NodeTreeItem {
    mesh?: string,
    position?: Vector3,
    pivot?: Vector3,
    axis?: Vector3,
    angleProperty?: string,
    angleOffset?: number,
    spinnerProperty?: string,
    spinnerSenseProperty?: string,
    children?: Array<NodeTreeItem>,
};

class Craft {
    rootEntity: Entity = null;
    craftEntities: Array<CraftEntity> = [];
    nodeTree: object = {};
    objFileInput: HTMLInputElement;
    
    constructor(scene: Scene, camera: OriginCamera) {
        this.rootEntity = createEntity('craftEntity', scene, camera);
        
        /**
         * TODO document the following and provide settings
         * ! The craft, after loading, should have the nose to +x, up to -z and left -y
         * To achieve this, do the following in blender:
         * 1. The craft has fwd pointing to -x, left is to -y, and bottom to -z
         * 2. Export the craft with the default blender export settings (+y forward, -z up)
         */

        this.rootEntity.doublepos = new Vector3(0, 0, EARTH_RADIUS);
        // this.rootEntity.setPivotPoint(new Vector3(0, 0, 0));

        this.loadDefinitionFile(scene);
        this.load3D_Models(scene);
    }

    update(craftData, deltaT: number, camera: OriginCamera) {
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

        console.log(craftData);

        // latitude -> North-south
        // longitude -> East-west
        const latitude_rad = craftData['latitude_rad'];
        const longitude_rad = craftData['longitude_rad'];

        const craftRadius = EARTH_RADIUS + craftData['altitude_m'];
        
        /**
         * Source coordinates: JSBSim
         * Target coordinates: viz
         */
        const craftCG = new Vector3(
            -craftData['cg_x_m'],
             craftData['cg_y_m'],
            -craftData['cg_z_m'],
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
            craftData['ecef_q_2'],
            craftData['ecef_q_3'],
            craftData['ecef_q_4'],
            craftData['ecef_q_1'],
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
            childEntity.updateFromSim(craftData, deltaT);
        }
    }

    loadDefinitionFile(scene: Scene) {
        const jsbsimDefInput: HTMLInputElement = document.querySelector('#jsbsim_def');
        jsbsimDefInput.addEventListener('change', (event: Event) => {
            const fileList = jsbsimDefInput.files;
            
            if (!fileList[0]) return;

            const reader = new FileReader();
            reader.readAsText(fileList[0], 'utf-8');
            reader.onload = (evt) => this.parseJSBSimDef(evt.target.result.toString(), scene);
            
        })
    }

    parseJSBSimDef(xmlText: string, scene: Scene) {
        const parser = new DOMParser();
        const xmlDoc = parser.parseFromString(xmlText, 'application/xml');

        const vizEl = xmlDoc.documentElement.getElementsByTagName('viz');

        if (!vizEl[0]) return;

        const nodeEls = vizEl[0].children;

        for (let i = 0; i < nodeEls.length; ++i) {
            if (nodeEls[i].nodeName === 'node') {
                this.nodeTree = this.parseJSBSimDefNode(nodeEls[i]);
            }
        }

        console.log(this.nodeTree);

        this.applyLoadedFiles(scene)
    }

    parseJSBSimDefNode(currentNode: Element): object {
        const toReturn: NodeTreeItem = {
            mesh: null,
            position: new Vector3(0, 0, 0),
            pivot: new Vector3(0, 0, 0),
            axis: new Vector3(1, 0, 0),
            spinnerProperty: null,
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
                toReturn.children.push(this.parseJSBSimDefNode(childEls[i]));
                break;
            case 'position':
                const positionEls = childEls[i].children;

                let posX, posY, posZ;
                for (let j = 0; j < positionEls.length; ++j) {
                    /**
                     * TODO maybe add position offset 
                     */
                    
                    if (positionEls[j].nodeName === 'x') posX = Number(positionEls[j].textContent);
                    if (positionEls[j].nodeName === 'y') posY = Number(positionEls[j].textContent);
                    if (positionEls[j].nodeName === 'z') posZ = Number(positionEls[j].textContent);
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
                toReturn.spinnerSenseProperty = childEls[i].textContent.trim();
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

    load3D_Models(scene: Scene) {
        this.objFileInput = document.querySelector('#objfile_input');
        this.objFileInput.addEventListener('change', (event: Event) => {
            this.applyLoadedFiles(scene);
        });
    }


    applyLoadedFiles(scene: Scene) {
        this.loadMeshEntityFromTree(scene, this.nodeTree, this.rootEntity);
    }

    async loadMeshEntityFromTree(scene: Scene, currentDefNode: NodeTreeItem, parentEntity: TransformNode) {
        const fileList = this.objFileInput.files;
        
        if (!currentDefNode) return;
        if (!fileList) return;
        
        console.log(currentDefNode.mesh || 'Loading node without mesh');

        const newCraftEntity = new CraftEntity('', scene);
        newCraftEntity.parent = parentEntity;
        // newCraftEntity.setposi = currentDefNode.position;
        newCraftEntity.setPivotPoint(currentDefNode.pivot);
        newCraftEntity.axis = currentDefNode.axis;
        newCraftEntity.angleProperty = currentDefNode.angleProperty;
        newCraftEntity.angleOffset = currentDefNode.angleOffset;
        newCraftEntity.spinnerProperty = currentDefNode.spinnerProperty;
        newCraftEntity.spinnerSenseProperty = currentDefNode.spinnerSenseProperty;
        this.craftEntities.push(newCraftEntity);

        if (currentDefNode.mesh) {
            for (let i = 0; i < fileList.length; ++i) {
                try {
                    const meshes = (await SceneLoader.ImportMeshAsync(currentDefNode.mesh, '', fileList.item(i), scene)).meshes;
                    if (meshes.length == 0) throw new Error(`Mesh ${currentDefNode.mesh} not found in ${fileList.item(i).name} trying next one`);

                    meshes[0].parent = newCraftEntity;
                    break;
                } catch (e) {
                    console.error(e);
                }
            }
        }

        
        if (currentDefNode.children) {
            for (const child of currentDefNode.children) {
                await this.loadMeshEntityFromTree(scene, child, newCraftEntity);
            }
        }
        
    }
}

export default Craft;
