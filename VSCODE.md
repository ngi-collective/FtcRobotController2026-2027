# Visual Studio Code

This repository can be edited in VS Code as well as Android Studio. Two files in `.vscode/`,
added in #21, set it up. Building, testing and installing still go through the `mise` tasks in a
terminal (see `CLAUDE.md`); VS Code's job is editing, navigation and error highlighting.

## What is committed

### `.vscode/extensions.json`

```json
{ "recommendations": ["vscjava.vscode-java-pack"] }
```

Recommends Microsoft's **Extension Pack for Java**: Red Hat's Java language support, which also
imports the Gradle project, plus the Java debugger and test runner. VS Code offers to install it
the first time the folder is opened.

### `.vscode/settings.json`

```json
{
  "java.configuration.updateBuildConfiguration": "automatic",
  "java.jdt.ls.androidSupport.enabled": "on"
}
```

- **`java.configuration.updateBuildConfiguration: automatic`.** Re-imports the Gradle project
  whenever a build file changes, without asking first. The other values are `interactive`, which
  asks on every change, and `disabled`. Automatic suits this repository because the ftc-sim plugin
  adds the `robot`/`simulated` flavours and the simulator's dependencies to TeamCode. Until VS Code
  re-imports after a build change, it does not know about them.
- **`java.jdt.ls.androidSupport.enabled: on`.** Turns on the Java extension's Android project
  import, which is how VS Code learns that `TeamCode` and `FtcRobotController` are Android modules
  and resolves `android.*` and the FTC SDK's AARs. The extension marks this as experimental, and its
  default, `auto`, enables it only in VS Code Insiders. `on` enables it in regular VS Code. It needs
  Android Gradle Plugin 3.2.0 or later; this repository uses 8.13.2.

## What VS Code needs from your machine

The Gradle import runs the project's own Gradle (9.1) and AGP (8.13.2). These need a JDK and the
Android SDK. `mise` supplies both, as `JAVA_HOME` (Temurin 21) and `ANDROID_HOME`:

- **Start VS Code from a terminal in this directory** with `mise` active, using `code .`, so it
  inherits those variables. If VS Code is already running, `code .` may open a window in the
  existing process, which keeps the environment it started with. Quit VS Code first.
- **Run `mise run setup-sdk` once** if you have not, so the SDK platform and build-tools exist
  under `ANDROID_HOME`.
- **`local.properties` overrides `ANDROID_HOME`.** If it has an `sdk.dir` line, Gradle uses that
  SDK in VS Code and on the command line alike. It is gitignored and machine-specific.
  `mise run verify-toolchain` reports which SDK each side is using.

The Java language server itself runs on the JRE bundled in the extension, so it does not need
`JAVA_HOME`. Only the Gradle import does.

## What it leaves behind

The Java extension compiles into a `bin/` folder in each module, such as `TeamCode/bin/`. These
are covered by the `bin/` rule already in `.gitignore`, and Gradle ignores them. If a module folder
is deleted while VS Code is open, the extension may recreate its empty `bin/` folders; close VS
Code before deleting one.
