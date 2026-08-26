// ================= PER-FRAME DYNAMIC ENTITY 3D SYNC =================
// Builds/updates the 3D mesh for each buoy/dynamic-boat entity in state.js's
// `entities` array (parked marina boats are excluded — they get their
// meshes once in scene-marina.js and never change). Called once per frame
// from main.js. Depends on scene-environment.js (`scene`) and state.js (`entities`).

// 3D Entity Tracker Map (High-Detail Obstacles)
const threeEntities = new Map();

function sync3DEntities() {
    entities.forEach(ent => {
        if (ent.isParkedShip) return; // Skip rendering yellow buoy rings for parked quay ships!
        if (!threeEntities.has(ent.id)) {
            let mesh;
            if (ent.type === 'static') {
                // High-detail Nautical Buoy
                const buoyGroup = new THREE.Group();
                const buoyGeo = new THREE.CylinderGeometry(0.55, 0.4, 2.2, 16);
                const buoyMat = new THREE.MeshLambertMaterial({ color: 0xff8c00 }); // Nautical Orange
                const buoyMesh = new THREE.Mesh(buoyGeo, buoyMat);
                buoyGroup.add(buoyMesh);

                // Dotted Safety Keep-Out Ring in 3D
                const ringGeo = new THREE.RingGeometry(5.8, 6.0, 32);
                const ringMat = new THREE.MeshBasicMaterial({ color: 0xffa500, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
                const ringMesh = new THREE.Mesh(ringGeo, ringMat);
                ringMesh.rotation.x = Math.PI / 2;
                ringMesh.position.y = -0.4;
                buoyGroup.add(ringMesh);

                // Blinking Beacon Light
                const light = new THREE.PointLight(0xff0000, 1.2, 12);
                light.position.set(0, 1.4, 0);
                buoyGroup.add(light);

                mesh = buoyGroup;
            } else if (ent.type === 'dynamic') {
                // Ultra-Realistic Dynamic Patrol Vessel Model
                const obsBoatGroup = new THREE.Group();

                // 1. Hydrodynamic Sleek V-Hull (Deep Navy / Marine Teal)
                const hullMat = new THREE.MeshStandardMaterial({ color: 0x0f4c5c, roughness: 0.15, metalness: 0.3 });
                const hullMesh = new THREE.Mesh(new THREE.BoxGeometry(3.8, 0.7, 1.5), hullMat);
                hullMesh.position.set(0, 0.2, 0);
                obsBoatGroup.add(hullMesh);

                // 2. Red Anti-Fouling Waterline Stripe
                const stripeMat = new THREE.MeshStandardMaterial({ color: 0xc1121f, roughness: 0.3 });
                const stripeMesh = new THREE.Mesh(new THREE.BoxGeometry(3.82, 0.22, 1.52), stripeMat);
                stripeMesh.position.set(0, 0.05, 0);
                obsBoatGroup.add(stripeMesh);

                // 3. Tapered Pointed Bow Nose
                const bowGeo = new THREE.ConeGeometry(0.75, 1.3, 4);
                const bowMesh = new THREE.Mesh(bowGeo, hullMat);
                bowMesh.rotation.x = Math.PI / 2;
                bowMesh.rotation.z = -Math.PI / 2;
                bowMesh.position.set(2.5, 0.2, 0);
                obsBoatGroup.add(bowMesh);

                // 4. White Superstructure Cabin Bridge & Tinted Glass
                const cabinBase = new THREE.Mesh(
                    new THREE.BoxGeometry(1.6, 0.85, 1.2),
                    new THREE.MeshStandardMaterial({ color: 0xf8f9fa, roughness: 0.1 })
                );
                cabinBase.position.set(-0.2, 0.825, 0);
                obsBoatGroup.add(cabinBase);

                // Tinted Glass Windshield
                const glassMat = new THREE.MeshStandardMaterial({ color: 0x1d2d44, metalness: 0.9, roughness: 0.05, transparent: true, opacity: 0.85 });
                const glassMesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.5, 1.15), glassMat);
                glassMesh.position.set(0.2, 0.98, 0);
                obsBoatGroup.add(glassMesh);

                // 5. Radar Arch & Rotating Radar Antenna Bar
                const archMat = new THREE.MeshStandardMaterial({ color: 0xd1d5db, metalness: 0.8, roughness: 0.2 });
                const mastLeft = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.9), archMat);
                mastLeft.position.set(-0.8, 1.45, 0.45);
                const mastRight = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.9), archMat);
                mastRight.position.set(-0.8, 1.45, -0.45);
                const archTop = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.08, 1.0), archMat);
                archTop.position.set(-0.8, 1.9, 0);
                obsBoatGroup.add(mastLeft, mastRight, archTop);

                // Rotating Radar Scanner
                const radarBar = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.08, 0.15), new THREE.MeshStandardMaterial({ color: 0xffffff }));
                radarBar.position.set(-0.8, 2.0, 0);
                radarBar.name = 'radarBar';
                obsBoatGroup.add(radarBar);

                // 6. Stainless Steel Bow Guard Railings
                const railMat = new THREE.MeshStandardMaterial({ color: 0xe5e7eb, metalness: 0.9, roughness: 0.1 });
                const railLeft = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1.8), railMat);
                railLeft.rotation.z = Math.PI / 2;
                railLeft.position.set(1.4, 0.7, 0.7);
                const railRight = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1.8), railMat);
                railRight.rotation.z = Math.PI / 2;
                railRight.position.set(1.4, 0.7, -0.7);
                obsBoatGroup.add(railLeft, railRight);

                // 7. Orange LifeBuoy Rescue Ring on Cabin Side
                const buoyRingGeo = new THREE.TorusGeometry(0.18, 0.05, 8, 16);
                const buoyRingMat = new THREE.MeshLambertMaterial({ color: 0xff5722 });
                const buoyRing = new THREE.Mesh(buoyRingGeo, buoyRingMat);
                buoyRing.position.set(-0.2, 0.9, 0.62);
                buoyRing.rotation.y = Math.PI / 2;
                obsBoatGroup.add(buoyRing);

                // 8. Navigation Lights (Port/Red on left, Starboard/Green on right)
                const portLight = new THREE.Mesh(
                    new THREE.SphereGeometry(0.08, 8, 8),
                    new THREE.MeshBasicMaterial({ color: 0xff0000 })
                );
                portLight.position.set(0.5, 1.05, 0.6);
                const stbdLight = new THREE.Mesh(
                    new THREE.SphereGeometry(0.08, 8, 8),
                    new THREE.MeshBasicMaterial({ color: 0x00ff00 })
                );
                stbdLight.position.set(0.5, 1.05, -0.6);
                obsBoatGroup.add(portLight, stbdLight);

                // 9. Dual Black Outboard Engines at Stern
                const engineMat = new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.8 });
                const eng1 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.85, 0.35), engineMat);
                eng1.position.set(-2.1, 0.2, 0.4);
                const eng2 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.85, 0.35), engineMat);
                eng2.position.set(-2.1, 0.2, -0.4);
                obsBoatGroup.add(eng1, eng2);

                // 10. Dynamic Trailing White Water Propeller Wake
                const wakeGeo = new THREE.PlaneGeometry(3.5, 1.8);
                const wakeMat = new THREE.MeshBasicMaterial({
                    color: 0xffffff,
                    transparent: true,
                    opacity: 0.45,
                    side: THREE.DoubleSide
                });
                const wakeMesh = new THREE.Mesh(wakeGeo, wakeMat);
                wakeMesh.rotation.x = -Math.PI / 2;
                wakeMesh.position.set(-3.6, -0.38, 0);
                obsBoatGroup.add(wakeMesh);

                mesh = obsBoatGroup;
            } else if (ent.type === 'goal') {
                const goalGeo = new THREE.TorusGeometry(1.5, 0.2, 16, 100);
                const goalMat = new THREE.MeshBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.8 });
                mesh = new THREE.Mesh(goalGeo, goalMat);
                mesh.rotation.x = Math.PI / 2;
                const glow = new THREE.PointLight(0x00ff00, 2, 20);
                glow.position.set(0, 2, 0);
                mesh.add(glow);
            }
            mesh.position.set(ent.ros_x, 0.5, -ent.ros_y);
            scene.add(mesh);
            threeEntities.set(ent.id, mesh);
        }
    });
}

