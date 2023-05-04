import {ShallowRef} from "vue";

export type DeepReadonly<T> = T extends Function ? T : T extends object ? { readonly [K in keyof T]: DeepReadonly<T[K]> } : T;
export type ShallowReadonlyRef<T> = ShallowRef<DeepReadonly<T>>;

export const sleep = (time: number) => {
    return new Promise((resolve) => setTimeout(resolve, time));
}

export const robotNameMap = (team: 'BLACK' | 'PURPLE', id: number) => {
    if (team === 'PURPLE') {
        return{
            1: "Wall-E",
            2: "R2D2",
            3: "Tron",
            4: "Marvin",
            5: "Jarvis",
            6: "Baymax",
            7: "Noo-Noo",
            8: "T-800",
            9: "K-9",
            10: "Bender",
            11: "Holt",
            12: "Chappie",
            13: "TARS",
            14: "",
            15: "Herman",
        }[id]
    }

    return ""
}