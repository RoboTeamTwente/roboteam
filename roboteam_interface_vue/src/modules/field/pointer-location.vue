<script setup lang="ts">

import {FederatedPointerEvent, Text} from "pixi.js";
import {CustomPixiApplication} from "./field-objects";
import {
    onBeforeUnmount, onUnmounted,
    shallowRef,
    watch
} from "vue";

// Reactive values
const
    props = defineProps<{
        app: CustomPixiApplication,
    }>(),
    cursorRef = shallowRef<Text | null>(null);

// Methods
const
    init = () => {
        console.log("Setting up pointer location");


        // Setup mouse position text
        props.app.stage.eventMode = 'static';
        props.app.stage.hitArea = props.app.screen;

        // Init cursor position text
        const cursor = new Text("", {fontSize: 16, fill: 'white'});
        cursor.x = props.app.screen.width * 0.025;
        cursor.y = props.app.screen.height * 0.025;
        props.app.stage.addChild(cursor);

        cursorRef.value = cursor;
        props.app.stage.addEventListener('pointerleave', onPointerLeave);
        props.app.stage.addEventListener('pointermove', onPointerMove);

    },
    onPointerLeave = (e: FederatedPointerEvent) => {
        cursorRef.value!.text = "";
    },
    onPointerMove = (e: FederatedPointerEvent) => {
        const pos = e.getLocalPosition(props.app.stage);
        cursorRef.value!.text = `[${((pos.x) / 100).toFixed(2)}x, ${((pos.y) / 100).toFixed(2)}y]`
    },
    cleanUp = () => {
        console.log("Cleaning up pointer location");
        props.app.stage.eventMode = 'none';
        props.app.stage.removeAllListeners('pointermove');
        props.app.stage.removeAllListeners('pointerleave');
        props.app.stage.removeEventListener('pointerleave', onPointerLeave);
        props.app.stage.removeEventListener('pointermove', onPointerMove);

        cursorRef.value?.destroy();
        cursorRef.value = null;
    };

watch(() => props.app, () => {
    cleanUp();
    init();
}, {immediate: true});
onUnmounted(cleanUp);
// onBeforeUnmount(cleanUp);
</script>

<template>
  <!-- Pointer Component (Nothing to render, rendering is done using pixi not Vue) -->
</template>