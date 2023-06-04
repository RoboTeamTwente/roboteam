<script setup lang="ts">
import { TabKey, TABS_DEFINITION } from '../../../tabs'

// @eslint-ignore vue/no-setup-props-destructure (this is should be fine as of Vue 3.3, https://blog.vuejs.org/posts/vue-3-3#reactive-props-destructure)
const { activeTab, tabs } = defineProps<{
  activeTab: TabKey
  tabs: TabKey[]
}>()

const emit = defineEmits<{
  (e: 'update:activeTab', payload: TabKey): void
}>()
</script>

<template>
  <div class="flex flex-col max-h-full">
    <div class="tabs rounded-none flex-nowrap overflow-auto whitespace-nowrap min-h-8">
      <a
        v-for="tab in tabs"
        class="tab tab-bordered gap-1 flex-nowrap"
        @click="emit('update:activeTab', tab)"
        :class="{ 'tab-active': activeTab === tab }"
        :key="tab"
      >
        <font-awesome-icon :icon="TABS_DEFINITION[tab].icon" />
        {{ tab }}
      </a>
      <div class="tab tab-bordered gap-1 flex-nowrap w-full" />
    </div>
    <div class="p-2 overflow-auto">
      <Component :is="TABS_DEFINITION[activeTab].component as any" />
    </div>
  </div>
</template>
