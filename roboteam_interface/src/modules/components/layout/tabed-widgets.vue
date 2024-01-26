<script setup lang="ts">
import { TabKey, TABS_DEFINITION } from '../../../tabs'

const props = defineProps<{
  activeTab: TabKey
  tabs: TabKey[]
}>()

const emit = defineEmits<{
  (e: 'update:activeTab', payload: TabKey): void
}>()
</script>

<template>
  <div class="flex flex-col max-h-full">
    <div
      class="tabs rounded-none flex-nowrap overflow-auto whitespace-nowrap min-h-8 subtile-scroll-bar"
    >
      <a
        v-for="tab in props.tabs"
        class="tab tab-bordered gap-1 flex-nowrap"
        @click="emit('update:activeTab', tab)"
        :class="{ 'tab-active': props.activeTab === tab }"
        :key="tab"
      >
        <font-awesome-icon :icon="TABS_DEFINITION[tab].icon" />
        {{ tab }}
      </a>
      <div class="tab tab-bordered gap-1 flex-nowrap w-full" />
    </div>
    <div class="p-2 overflow-auto">
      <Component :is="TABS_DEFINITION[props.activeTab].component as any" />
    </div>
  </div>
</template>
