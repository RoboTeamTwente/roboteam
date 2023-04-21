fn main() {
    let mut cfg = prost_build::Config::new();

    let classes = [
        // "proto.State",
        "proto.World",
        "proto.WorldBall",
        "proto.Vector2f",
        "proto.WorldRobot",
        "proto.RobotWheel",
        "proto.FeedbackOnlyRobot",

        "proto.RobotParameters",
        "proto.TeamParameters",
        "proto.SslDetectionBall",
        "proto.SslDetectionRobot",
        "proto.SslDetectionFrame",
        "proto.SslFieldLineSegment",

        "proto.RobotProcessedFeedback",
        "proto.SslGeometryData",
        "SslGeometryData",
        "proto.RobotWheels",
        // "proto.SslGeometryData",
        // "proto.SslDetectionRobot",
        // "proto.SslDetectionRobot",
        // "proto.SslDetectionRobot",
        // "proto.SslDetectionRobot",
        // "proto.SslDetectionRobot",
        // "proto.SslDetectionRobot",

    ];
    for class in classes {
        cfg.type_attribute(class, "#[derive(serde::Serialize, serde::Deserialize)]");
        cfg.type_attribute(class, "#[serde(default)]");
    }

    cfg.compile_protos(&["src/proto/State.proto", ], &["src/proto"])
        .expect("Proto file build failed");

    tauri_build::build();
}
