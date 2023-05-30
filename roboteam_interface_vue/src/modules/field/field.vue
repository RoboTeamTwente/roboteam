<script setup lang="ts">

import {CustomPixiApplication, FieldDrawing} from "./field-objects";
import {onUnmounted, shallowRef, toRaw, watch} from "vue";
import {proto} from "../../generated/proto";
import ISSL_GeometryFieldSize = proto.ISSL_GeometryFieldSize;


// Reactive values
const props = defineProps<{
        app: CustomPixiApplication,
        isYellow: boolean,
        fieldGeometry: ISSL_GeometryFieldSize | null,
    }>(),
    field = shallowRef<FieldDrawing | null>(null);

// Methods
const
    init = () => {
        if (props.fieldGeometry === null) { return; }
        const fieldDrawing = new FieldDrawing({
            fieldGeometry: props.fieldGeometry,
            isYellow: props.isYellow,
        });

        props.app.drawingsContainer.addChild(fieldDrawing);
        field.value = fieldDrawing;

        console.log(props.app);
        console.log(field.value)
    },
    cleanUp = () => {
        console.log("Cleaning up field");
        field.value?.destroy({children: true});
        field.value = null;
    };

watch([
    () => props.app,
    () => props.fieldGeometry,
    () => props.isYellow
], () => {
    cleanUp();
    init();
}, {immediate: true});
onUnmounted(cleanUp);

</script>

<template>
    <!-- Field Component -->
    <slot v-if="field !== null"/>
</template>