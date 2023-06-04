import { createApp } from 'vue'

import './index.css'
import App from './app.vue'

const app = createApp(App)

import { library } from '@fortawesome/fontawesome-svg-core'

/* import font awesome icon component */
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'

/* import specific icons */
import { fas } from '@fortawesome/free-solid-svg-icons'
import { far } from '@fortawesome/free-regular-svg-icons'
import { createPinia } from 'pinia'
import piniaPluginPersistedstate from 'pinia-plugin-persistedstate'

/* add icons to the library */
library.add(fas, far)

const pinia = createPinia()
pinia.use(piniaPluginPersistedstate)

app.component('font-awesome-icon', FontAwesomeIcon).use(pinia).mount('#app')
