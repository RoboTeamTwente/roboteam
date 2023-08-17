# RTT Interface
Stack: Vite.js + Vue 3 + Tauri 

## Installation
Run `yarn install`

## Development
There are two ways you can develo your app.

### In Browser
- `yarn serve`
  - launches vite and you can test and develop your app in the browser at http://localhost:8080.

### In Tauri Window

Launch two terminals and in

1- `yarn serve:tauri`

This launches Vite and configures [Unified Network](https://github.com/yooneskh/unified-network) (which is mine) to use Tauri for api calls (to get around CORS problems).

2- `yarn serve:native`

This launches Tauri window and you would see your app in the native window.

**Note:** There are mainly 2 differences between development in browser and in Tauri window.

- One is who executes your http calls, because when in browser, you are subject to CORS rules, but when testing in Tauri mode, Tauri's native module is executing the http calls so CORS will not be a problem.

- Second is the renderer engine. In browsers, it is usually the latest modern engine, but in Tauri, it will be the OS's web engine, which is good, but maybe not as good as the browsers.

## Building

`yarn build` builds web application and packages them with Tauri in "./src-tauri/target/releases".