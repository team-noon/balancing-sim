import * as fs from "jsr:@std/fs";

const worldInfos = [
  "trainDebug",
  "train",
  "animator",
  "walker",
  "inference",
] as const;

const folderPath = import.meta.dirname + "/worlds";

if (fs.existsSync(folderPath)) {
  const resp = prompt(
    "Are you sure you want to continue? [y/N]\nA worlds folder already exists, this will delete it if you continue",
  )?.toLowerCase();

  if (!(resp === "y" || resp === "yes")) {
    Deno.exit();
  }
}

try {
  Deno.removeSync(folderPath, { recursive: true });
} catch {/**/}

fs.ensureDirSync(folderPath);

for (const worldInfo of worldInfos) {
  const extern = [
    "#VRML_SIM R2025a utf8",
    "",
    'EXTERNPROTO "../protos/noonRobot.proto"',
  ];

  if (worldInfo !== "train") {
    extern.push(
      'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"',
      'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"',
      'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"',
    );
  }

  if (worldInfo === "trainDebug" || worldInfo === "train") {
    extern.push('EXTERNPROTO "../protos/noonRobotSpawner.proto"');
  }

  const worldInfoBlock = `DEF WorldInfo WorldInfo {
  ERP 0.3
  basicTimeStep 16
  physicsDisableLinearThreshold 0.005
  physicsDisableAngularThreshold 0.005
  contactProperties [
    ContactProperties {
      material2 "floor"
      coulombFriction [
        1.2
      ]
      rollingFriction 0.1 0.1 0.1
      bounce 0
      bounceVelocity 0
      forceDependentSlip [
        0.001
      ]
      softERP 0.5
    }
  ]
}`;

  const viewpoint = worldInfo === "train" ? "" : `Viewpoint {
  orientation 0.12 0.14 -1 1.5
  position -0.5 8 2
}

TexturedBackgroundLight {
}

TexturedBackground {
}`;

  const floor = `Floor {
  contactMaterial "floor"
  size ${worldInfo == "train" ? "2000 2000" : "100 100"}
  ${worldInfo == "train" ? "appearance NULL" : ""}
  }`;

  let robot = "";

  switch (worldInfo) {
    case "animator":
      robot = `noonRobot {
  controller "animator"
  cylinderSubdivision 16
}`;
      break;

    case "walker":
      robot = `noonRobot {
  controller "walker"
  cylinderSubdivision 16
}`;
      break;

    case "inference":
      robot = `noonRobot {
  controller "<extern>"
  inference TRUE
}`;
      break;

    case "trainDebug":
      robot = `noonRobot {
  controller "<extern>"
  name "trainer"
}

DEF TRAINER noonRobotSpawner {
  count 0
  spacing 30
}`;
      break;

    case "train":
      robot = `noonRobot {
  controller "<extern>"
  name "trainer"
}

DEF TRAINER noonRobotSpawner {
  count 0
  spacing 400
}`;
      break;
  }

  const content = [
    ...extern,
    "",
    worldInfoBlock,
    "",
    viewpoint,
    "",
    floor,
    "",
    robot,
  ].join("\n");

  await Deno.writeTextFile(`${folderPath}/${worldInfo}.wbt`, content);
}

// i wrote the logic but chatgpt refactored it