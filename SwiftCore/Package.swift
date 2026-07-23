// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "ICR8600Core",
    platforms: [.macOS(.v13)],
    products: [
        .library(name: "ICR8600Core", type: .static, targets: ["ICR8600Core"])
    ],
    targets: [
        .target(
            name: "CICR8600Core",
            path: "Sources/CICR8600Core"
        ),
        .target(
            name: "ICR8600Core",
            dependencies: ["CICR8600Core"],
            path: "Sources/ICR8600Core",
            linkerSettings: [
                .linkedFramework("IOKit"),
                .linkedFramework("IOUSBHost")
            ]
        )
    ]
)
