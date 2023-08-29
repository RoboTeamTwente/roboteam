module.exports = {
  packagerConfig: {
    asar: true,
    icon: 'electron/icons/icon',
  },
  rebuildConfig: {},
  makers: [
    {
      name: '@electron-forge/maker-squirrel',
      config: {},
    },
    {
      name: '@electron-forge/maker-zip',
      platforms: ['darwin'],
    },
    {
      name: '@electron-forge/maker-deb',
      config: {
        options: {
          icon: '/electron/icons/128x128.png'
        }
      },
    },
    {
      name: '@electron-forge/maker-rpm',
      config: {},
    },
  ],
  plugins: [
    {
      name: '@electron-forge/plugin-auto-unpack-natives',
      config: {},
    },
    {
      name: '@electron-forge/plugin-vite',
      config: {
        // `build` can specify multiple entry builds, which can be
        // Main process, Preload scripts, eWorker process, etc.
        build: [
          {
            // `entry` is an alias for `build.lib.entry`
            // in the corresponding file of `config`.
            entry: 'electron/main.ts',
            config: 'electron/config/vite.main.config.ts'
          },
          {
            entry: 'electron/preload.ts',
            config: 'electron/config/vite.preload.config.ts'
          }
        ],
        renderer: [
          {
            name: 'main_window',
            config: 'vite.config.ts'
          }
        ]
      }
    }
  ],
};
