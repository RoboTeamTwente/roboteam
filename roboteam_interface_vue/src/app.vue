<script setup lang='ts'>

import {ref} from "vue";
import GameCanvas from "./modules/field/game-canvas.vue";
import {useUIStore} from "./modules/stores/ui-store";
import TopPanel from "./modules/components/layout/top-panel.vue";
import {useAIClient} from "./services/ai-client";
import TabedWidgets from "./modules/components/layout/tabed-widgets.vue";
import {useVisionDataStore} from "./modules/stores/data-stores/vision-data-store";
import SidebarResizer from "./modules/components/layout/sidebar-resizer.vue";

let url = ref("localhost:12676");


let aiClient = useAIClient();
const uiStore = useUIStore();
const visionData = useVisionDataStore();

//
// icon name for status open: check

</script>

<template>
    <div class="modal" :class="{'modal-open': aiClient.status.value !== 'OPENED'}">
        <div class="modal-box flex flex-col gap-4">
            <div class="m-auto">
                <img src="/favicon.svg" class="w-32"/>
            </div>


            <span class="flex items-center">Status: <span class="inline-block rounded-full w-2 h-2 bg-red-600 ml-2 mr-1" :class="{
                'bg-green-600': aiClient.status.value == 'OPENED'
            }"/> {{ aiClient.status.value }}</span>
            <div class="form-control w-full max-w">
                <label class="label">
                    <span class="label-text">AI InterfaceGateway url</span>
                </label>
                <label class="input-group">
                <span>ws://</span>
                <input class="input input-bordered w-full" v-model="url">
                </label>
            </div>
            <button class="btn btn-primary" @click="() => aiClient.open( 'ws://' + url)">Connect</button>
        </div>
    </div>

    <div class="contents" :style="{
        '--left-bar-width': `${uiStore.leftPanel.size}px`,
        '--bottom-bar-height': `${uiStore.bottomPanel.size}px`
    }">
        <div class="scaffold h-screen w-screen">
            <top-panel/>

            <!-- Left Sidebar START -->
            <sidebar-resizer
                    direction="x"
                    class="left-sidebar-resize"
                    @drag="uiStore.resizeLeftPanel(uiStore.leftPanel.size - $event.x)"
            />
            <Transition name="left-sidebar" :duration="300">
                <div class="left-sidebar" v-if="!uiStore.$state.leftPanel.collapsed">
                    <tabed-widgets v-model:active-tab="uiStore.leftPanel.selectedTab" :tabs="uiStore.leftPanel.selectableTabs"/>
<!--                    <tabed-widgets v-model:active-tab="uiStore.leftPanel.selectedTab" :tabs="TABS"/>-->
                </div>
            </Transition>
            <!-- Left Sidebar END -->

            <!-- Game Canvas START -->
            <main class="game-canvas flex flex-col justify-center bg-base-200 min-h-0 p-4" :class="{
                  '!justify-start': visionData.latestWorld == null,
                }">
                <game-canvas v-if="visionData.latestWorld !== null"/>
                <div v-else class="alert alert-warning justify-start">
                    <font-awesome-icon icon="fa-circle-exclamation"/>
                    No field data
                </div>
            </main>
            <!-- Game Canvas END -->

            <!-- Bottom sidebar START -->
            <sidebar-resizer
                    direction="y"
                    class="bottom-sidebar-resize"
                    @drag="uiStore.resizeBottomPanel(uiStore.bottomPanel.size + $event.y)"
            />
            <Transition name="bottom-sidebar" :duration="300">
                <div class="bottom-sidebar" v-if="!uiStore.$state.bottomPanel.collapsed">
                    <tabed-widgets v-model:active-tab="uiStore.bottomPanel.selectedTab" :tabs="uiStore.bottomPanel.selectableTabs"/>
                </div>
            </Transition>
            <!-- Bottom sidebar END -->
        </div>
    </div>
</template>

<style>


body {
    --left-bar-width: 320px;
    --bottom-bar-height: 320px;
}

.scaffold {
    display: grid;
    grid-template-areas:
        'tpa   tpa   tpa   tpa'
        'lsb   lbr   gac   gac'
        'lsb   lbr   bbr   bbr'
        'lsb   lbr   bsb   bsb';

    grid-template-columns: min(90vw, var(--left-bar-width)) 3px 1fr 1fr auto;
    grid-template-rows: 3rem 1fr 3px min(80vh, var(--bottom-bar-height)) auto;
}

.top-panel {
    grid-area: tpa;
}

.left-sidebar {
    grid-area: lsb;
}

.left-sidebar-resize {
    grid-area: lbr;
}

.game-canvas {
    grid-area: gac;
}

.bottom-sidebar {
    grid-area: bsb;
}

.bottom-sidebar-resize {
    grid-area: bbr;
}

.scaffold:has(.left-sidebar-enter-active, .left-sidebar-leave-active, .bottom-sidebar-enter-active, .bottom-sidebar-leave-active) {
    transition: all 400ms;
}

.scaffold:has(.left-sidebar-enter-from) {
    --left-bar-width: 0px !important;
}

.scaffold:has(.left-sidebar-leave-to) {
    --left-bar-width: 0px !important;
}

.scaffold:not(:has(.left-sidebar)) {
    --left-bar-width: 0px !important;
}

.scaffold:has(.bottom-sidebar-enter-from) {
    --bottom-bar-height: 0px !important;
}

.scaffold:has(.bottom-sidebar-leave-to) {
    --bottom-bar-height: 0px !important;
}

.scaffold:not(:has(.bottom-sidebar)) {
    --bottom-bar-height: 0px !important;
}

</style>