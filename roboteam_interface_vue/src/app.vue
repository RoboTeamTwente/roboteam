<script setup lang='ts'>

import {computed, ref} from "vue";
import GameCanvas from "./modules/field/game-canvas.vue";
import {useUIStore} from "./modules/stores/ui-store";
import TopPanel from "./modules/components/layout/top-panel.vue";
import {useAIClient} from "./services/ai-client";
import ResizableSlider from "./modules/components/layout/panel-slider.vue";
import PanelSlider from "./modules/components/layout/panel-slider.vue";
import TabedWidgets from "./modules/components/layout/tabed-widgets.vue";
import {useVisionDataStore} from "./modules/stores/dataStores/vision-data-store";
import {useProtoWebSocket} from "./utils";

let url = ref("ws://localhost:12676");
// const {isReady, data, send, open} = useProtoWebSocket();

let aiClient = useAIClient();
const uiStore = useUIStore();
const visionData = useVisionDataStore();

const gridElement = ref<null | HTMLElement>(null);


</script>

<template>
  <div class="modal" :class="{'modal-open': aiClient.status.value !== 'OPENED'}">
    <div class="modal-box flex flex-col gap-4">
        Status: {{aiClient.status.value}}
        <div class="form-control w-full max-w">
            <label class="label">
                <span class="label-text">AI InterfaceGateway url</span>
            </label>
            <input class="input input-bordered" v-model="url">
        </div>
        <button class="btn btn-primary" @click="() => aiClient.open(url)">Connect</button>
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

    <main class="grid-in-main flex flex-col justify-center bg-base-200 min-h-0 p-4" :class="{
      '!justify-start': visionData.latestWorld == null,
    }">
      <game-canvas v-if="visionData.latestWorld !== null"/>
        <div class="alert alert-warning" v-else>
            <div>
                <font-awesome-icon icon="fa-circle-exclamation" />
                <span>No field data</span>
            </div>
        </div>
    </main>

    <panel-slider direction="y" class="grid-in-drag-y"
                  @drag="uiStore.resizeBottomPanel(uiStore.bottomPanel.size + $event.y)"/>

    <div class="grid-in-footer bg-base-100 flex flex-col" v-if="!uiStore.$state.bottomPanel.collapsed">
      <tabed-widgets />
    </div>
  </div>
</template>