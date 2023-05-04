<script setup lang='ts'>

import {computed, ref} from "vue";
import GameCanvas from "./modules/field/game-canvas.vue";
import {useUIStore} from "./modules/stores/ui-store";
import TopPanel from "./modules/components/layout/top-panel.vue";
import {useAIClient} from "./services/ai-client";
import ResizableSlider from "./modules/components/layout/panel-slider.vue";
import PanelSlider from "./modules/components/layout/panel-slider.vue";
import TabedWidgets from "./modules/components/layout/tabed-widgets.vue";
import {useGameControllerStore} from "./modules/stores/ai-store";
import {useVisionDataStore} from "./modules/stores/dataStores/vision-data-store";


let {status} = useAIClient("ws://localhost:12676");
const uiStore = useUIStore();
const visionData = useVisionDataStore();

const gridElement = ref<null | HTMLElement>(null);


</script>

<template>
  <div class="modal" :class="{'modal-open': status !== 'OPEN',}">
    <div class="modal-box">
      <h3 class="font-bold text-lg">Waiting for connection to the AI</h3>
      <p>[status: {{status}}]</p>
    </div>
  </div>

  <div ref="gridElement" class="grid grid-areas-layout grid-cols-layout grid-rows-layout h-screen w-screen"
       :style="{'--left-bar-width': `${uiStore.leftPanelSize}px`, '--bottom-bar-height': `${uiStore.bottomPanelSize}px`}">

    <top-panel/>

    <resizable-slider direction="x" class="grid-in-drag-x"
                      @drag="uiStore.resizeLeftPanel(uiStore.leftPanel.size - $event.x)"/>

    <div class="grid-in-nav bg-base bg-base-100" v-if="!uiStore.$state.leftPanel.collapsed">
     <tabed-widgets />
    </div>

    <main class="grid-in-main flex flex-col justify-center bg-base-200 min-h-0 p-4">
      <game-canvas v-if="visionData.latestWorld !== null"/>
    </main>

    <panel-slider direction="y" class="grid-in-drag-y"
                  @drag="uiStore.resizeBottomPanel(uiStore.bottomPanel.size + $event.y)"/>

    <div class="grid-in-footer bg-base-100 flex flex-col" v-if="!uiStore.$state.bottomPanel.collapsed">
      <tabed-widgets />
    </div>
  </div>
</template>