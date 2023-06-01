<script setup lang="ts">
import {Tab} from "../../stores/ui-store";

const props = defineProps<{
    activeTab: Tab,
    tabs: Tab[]
}>();

const emit = defineEmits<{
    (e: 'update:activeTab', payload: Tab): void
}>()

</script>

<template>
    <div class="flex flex-col max-h-full">
        <div class="tabs rounded-none flex-nowrap overflow-auto whitespace-nowrap min-h-8">
            <a v-for="tab in tabs" class="tab tab-bordered gap-1 flex-nowrap" @click="emit('update:activeTab', tab)"
               :class="{'tab-active': activeTab.name === tab.name}">
                <font-awesome-icon :icon="tab.icon"/>
                {{ tab.name }}
            </a>
            <div class="tab tab-bordered gap-1 flex-nowrap w-full "/>
        </div>
        <div class="p-2 overflow-auto">
            <Component :is="activeTab.component"/>
        </div>
    </div>
</template>