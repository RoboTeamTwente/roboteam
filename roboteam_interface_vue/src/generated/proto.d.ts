import * as $protobuf from "protobufjs";
import Long = require("long");
/** Namespace proto. */
export namespace proto {

    /** Properties of a RobotPath. */
    interface IRobotPath {

        /** RobotPath robotId */
        robotId?: (number|null);

        /** RobotPath points */
        points?: (proto.RobotPath.IPoint[]|null);
    }

    /** Represents a RobotPath. */
    class RobotPath implements IRobotPath {

        /**
         * Constructs a new RobotPath.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotPath);

        /** RobotPath robotId. */
        public robotId: number;

        /** RobotPath points. */
        public points: proto.RobotPath.IPoint[];

        /**
         * Creates a new RobotPath instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotPath instance
         */
        public static create(properties?: proto.IRobotPath): proto.RobotPath;

        /**
         * Encodes the specified RobotPath message. Does not implicitly {@link proto.RobotPath.verify|verify} messages.
         * @param message RobotPath message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotPath, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotPath message, length delimited. Does not implicitly {@link proto.RobotPath.verify|verify} messages.
         * @param message RobotPath message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotPath, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotPath message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotPath
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotPath;

        /**
         * Decodes a RobotPath message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotPath
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotPath;

        /**
         * Verifies a RobotPath message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotPath message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotPath
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotPath;

        /**
         * Creates a plain object from a RobotPath message. Also converts values to other types if specified.
         * @param message RobotPath
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotPath, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotPath to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotPath
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace RobotPath {

        /** Properties of a Point. */
        interface IPoint {

            /** Point x */
            x?: (number|null);

            /** Point y */
            y?: (number|null);
        }

        /** Represents a Point. */
        class Point implements IPoint {

            /**
             * Constructs a new Point.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.RobotPath.IPoint);

            /** Point x. */
            public x: number;

            /** Point y. */
            public y: number;

            /**
             * Creates a new Point instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Point instance
             */
            public static create(properties?: proto.RobotPath.IPoint): proto.RobotPath.Point;

            /**
             * Encodes the specified Point message. Does not implicitly {@link proto.RobotPath.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.RobotPath.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Point message, length delimited. Does not implicitly {@link proto.RobotPath.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.RobotPath.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Point message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotPath.Point;

            /**
             * Decodes a Point message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotPath.Point;

            /**
             * Verifies a Point message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Point message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Point
             */
            public static fromObject(object: { [k: string]: any }): proto.RobotPath.Point;

            /**
             * Creates a plain object from a Point message. Also converts values to other types if specified.
             * @param message Point
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.RobotPath.Point, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Point to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Point
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a RobotSTP. */
    interface IRobotSTP {

        /** RobotSTP robotId */
        robotId?: (number|null);

        /** RobotSTP role */
        role?: (string|null);

        /** RobotSTP roleStatus */
        roleStatus?: (string|null);

        /** RobotSTP tactic */
        tactic?: (string|null);

        /** RobotSTP tacticStatus */
        tacticStatus?: (string|null);

        /** RobotSTP skill */
        skill?: (string|null);

        /** RobotSTP skillStatus */
        skillStatus?: (string|null);
    }

    /** Represents a RobotSTP. */
    class RobotSTP implements IRobotSTP {

        /**
         * Constructs a new RobotSTP.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotSTP);

        /** RobotSTP robotId. */
        public robotId: number;

        /** RobotSTP role. */
        public role: string;

        /** RobotSTP roleStatus. */
        public roleStatus: string;

        /** RobotSTP tactic. */
        public tactic: string;

        /** RobotSTP tacticStatus. */
        public tacticStatus: string;

        /** RobotSTP skill. */
        public skill: string;

        /** RobotSTP skillStatus. */
        public skillStatus: string;

        /**
         * Creates a new RobotSTP instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotSTP instance
         */
        public static create(properties?: proto.IRobotSTP): proto.RobotSTP;

        /**
         * Encodes the specified RobotSTP message. Does not implicitly {@link proto.RobotSTP.verify|verify} messages.
         * @param message RobotSTP message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotSTP, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotSTP message, length delimited. Does not implicitly {@link proto.RobotSTP.verify|verify} messages.
         * @param message RobotSTP message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotSTP, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotSTP message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotSTP
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotSTP;

        /**
         * Decodes a RobotSTP message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotSTP
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotSTP;

        /**
         * Verifies a RobotSTP message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotSTP message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotSTP
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotSTP;

        /**
         * Creates a plain object from a RobotSTP message. Also converts values to other types if specified.
         * @param message RobotSTP
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotSTP, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotSTP to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotSTP
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a AIData. */
    interface IAIData {

        /** AIData robotStps */
        robotStps?: (proto.IRobotSTP[]|null);

        /** AIData robotPaths */
        robotPaths?: (proto.IRobotPath[]|null);
    }

    /** Represents a AIData. */
    class AIData implements IAIData {

        /**
         * Constructs a new AIData.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IAIData);

        /** AIData robotStps. */
        public robotStps: proto.IRobotSTP[];

        /** AIData robotPaths. */
        public robotPaths: proto.IRobotPath[];

        /**
         * Creates a new AIData instance using the specified properties.
         * @param [properties] Properties to set
         * @returns AIData instance
         */
        public static create(properties?: proto.IAIData): proto.AIData;

        /**
         * Encodes the specified AIData message. Does not implicitly {@link proto.AIData.verify|verify} messages.
         * @param message AIData message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IAIData, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified AIData message, length delimited. Does not implicitly {@link proto.AIData.verify|verify} messages.
         * @param message AIData message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IAIData, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a AIData message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns AIData
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.AIData;

        /**
         * Decodes a AIData message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns AIData
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.AIData;

        /**
         * Verifies a AIData message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a AIData message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns AIData
         */
        public static fromObject(object: { [k: string]: any }): proto.AIData;

        /**
         * Creates a plain object from a AIData message. Also converts values to other types if specified.
         * @param message AIData
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.AIData, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this AIData to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for AIData
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a GameSettings. */
    interface IGameSettings {

        /** GameSettings isPrimaryAi */
        isPrimaryAi?: (boolean|null);

        /** GameSettings isYellow */
        isYellow?: (boolean|null);

        /** GameSettings isLeft */
        isLeft?: (boolean|null);

        /** GameSettings robotHubMode */
        robotHubMode?: (proto.GameSettings.RobotHubMode|null);
    }

    /** Represents a GameSettings. */
    class GameSettings implements IGameSettings {

        /**
         * Constructs a new GameSettings.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IGameSettings);

        /** GameSettings isPrimaryAi. */
        public isPrimaryAi: boolean;

        /** GameSettings isYellow. */
        public isYellow: boolean;

        /** GameSettings isLeft. */
        public isLeft: boolean;

        /** GameSettings robotHubMode. */
        public robotHubMode: proto.GameSettings.RobotHubMode;

        /**
         * Creates a new GameSettings instance using the specified properties.
         * @param [properties] Properties to set
         * @returns GameSettings instance
         */
        public static create(properties?: proto.IGameSettings): proto.GameSettings;

        /**
         * Encodes the specified GameSettings message. Does not implicitly {@link proto.GameSettings.verify|verify} messages.
         * @param message GameSettings message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IGameSettings, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified GameSettings message, length delimited. Does not implicitly {@link proto.GameSettings.verify|verify} messages.
         * @param message GameSettings message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IGameSettings, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a GameSettings message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns GameSettings
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameSettings;

        /**
         * Decodes a GameSettings message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns GameSettings
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameSettings;

        /**
         * Verifies a GameSettings message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a GameSettings message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns GameSettings
         */
        public static fromObject(object: { [k: string]: any }): proto.GameSettings;

        /**
         * Creates a plain object from a GameSettings message. Also converts values to other types if specified.
         * @param message GameSettings
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.GameSettings, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this GameSettings to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for GameSettings
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace GameSettings {

        /** RobotHubMode enum. */
        enum RobotHubMode {
            UNKNOWN = 0,
            BASESTATION = 1,
            SIMULATOR = 2
        }
    }

    /** Properties of a Handshake. */
    interface IHandshake {

        /** Handshake moduleName */
        moduleName?: (string|null);

        /** Handshake declarations */
        declarations?: (proto.IUiOptionDeclarations|null);

        /** Handshake values */
        values?: (proto.IUiValues|null);
    }

    /** Represents a Handshake. */
    class Handshake implements IHandshake {

        /**
         * Constructs a new Handshake.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IHandshake);

        /** Handshake moduleName. */
        public moduleName: string;

        /** Handshake declarations. */
        public declarations?: (proto.IUiOptionDeclarations|null);

        /** Handshake values. */
        public values?: (proto.IUiValues|null);

        /**
         * Creates a new Handshake instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Handshake instance
         */
        public static create(properties?: proto.IHandshake): proto.Handshake;

        /**
         * Encodes the specified Handshake message. Does not implicitly {@link proto.Handshake.verify|verify} messages.
         * @param message Handshake message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IHandshake, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Handshake message, length delimited. Does not implicitly {@link proto.Handshake.verify|verify} messages.
         * @param message Handshake message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IHandshake, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Handshake message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Handshake
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Handshake;

        /**
         * Decodes a Handshake message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Handshake
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Handshake;

        /**
         * Verifies a Handshake message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Handshake message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Handshake
         */
        public static fromObject(object: { [k: string]: any }): proto.Handshake;

        /**
         * Creates a plain object from a Handshake message. Also converts values to other types if specified.
         * @param message Handshake
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Handshake, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Handshake to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Handshake
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Slider. */
    interface ISlider {

        /** Slider text */
        text?: (string|null);

        /** Slider min */
        min?: (number|null);

        /** Slider max */
        max?: (number|null);

        /** Slider interval */
        interval?: (number|null);

        /** Slider default */
        "default"?: (number|null);

        /** Slider dpi */
        dpi?: (number|null);
    }

    /** Represents a Slider. */
    class Slider implements ISlider {

        /**
         * Constructs a new Slider.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISlider);

        /** Slider text. */
        public text: string;

        /** Slider min. */
        public min: number;

        /** Slider max. */
        public max: number;

        /** Slider interval. */
        public interval: number;

        /** Slider default. */
        public default: number;

        /** Slider dpi. */
        public dpi: number;

        /**
         * Creates a new Slider instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Slider instance
         */
        public static create(properties?: proto.ISlider): proto.Slider;

        /**
         * Encodes the specified Slider message. Does not implicitly {@link proto.Slider.verify|verify} messages.
         * @param message Slider message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISlider, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Slider message, length delimited. Does not implicitly {@link proto.Slider.verify|verify} messages.
         * @param message Slider message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISlider, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Slider message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Slider
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Slider;

        /**
         * Decodes a Slider message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Slider
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Slider;

        /**
         * Verifies a Slider message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Slider message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Slider
         */
        public static fromObject(object: { [k: string]: any }): proto.Slider;

        /**
         * Creates a plain object from a Slider message. Also converts values to other types if specified.
         * @param message Slider
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Slider, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Slider to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Slider
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Checkbox. */
    interface ICheckbox {

        /** Checkbox text */
        text?: (string|null);
    }

    /** Represents a Checkbox. */
    class Checkbox implements ICheckbox {

        /**
         * Constructs a new Checkbox.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ICheckbox);

        /** Checkbox text. */
        public text: string;

        /**
         * Creates a new Checkbox instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Checkbox instance
         */
        public static create(properties?: proto.ICheckbox): proto.Checkbox;

        /**
         * Encodes the specified Checkbox message. Does not implicitly {@link proto.Checkbox.verify|verify} messages.
         * @param message Checkbox message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ICheckbox, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Checkbox message, length delimited. Does not implicitly {@link proto.Checkbox.verify|verify} messages.
         * @param message Checkbox message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ICheckbox, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Checkbox message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Checkbox
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Checkbox;

        /**
         * Decodes a Checkbox message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Checkbox
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Checkbox;

        /**
         * Verifies a Checkbox message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Checkbox message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Checkbox
         */
        public static fromObject(object: { [k: string]: any }): proto.Checkbox;

        /**
         * Creates a plain object from a Checkbox message. Also converts values to other types if specified.
         * @param message Checkbox
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Checkbox, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Checkbox to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Checkbox
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Dropdown. */
    interface IDropdown {

        /** Dropdown text */
        text?: (string|null);

        /** Dropdown options */
        options?: (string[]|null);
    }

    /** Represents a Dropdown. */
    class Dropdown implements IDropdown {

        /**
         * Constructs a new Dropdown.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IDropdown);

        /** Dropdown text. */
        public text: string;

        /** Dropdown options. */
        public options: string[];

        /**
         * Creates a new Dropdown instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Dropdown instance
         */
        public static create(properties?: proto.IDropdown): proto.Dropdown;

        /**
         * Encodes the specified Dropdown message. Does not implicitly {@link proto.Dropdown.verify|verify} messages.
         * @param message Dropdown message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IDropdown, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Dropdown message, length delimited. Does not implicitly {@link proto.Dropdown.verify|verify} messages.
         * @param message Dropdown message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IDropdown, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Dropdown message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Dropdown
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Dropdown;

        /**
         * Decodes a Dropdown message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Dropdown
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Dropdown;

        /**
         * Verifies a Dropdown message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Dropdown message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Dropdown
         */
        public static fromObject(object: { [k: string]: any }): proto.Dropdown;

        /**
         * Creates a plain object from a Dropdown message. Also converts values to other types if specified.
         * @param message Dropdown
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Dropdown, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Dropdown to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Dropdown
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RadioButton. */
    interface IRadioButton {

        /** RadioButton options */
        options?: (string[]|null);
    }

    /** Represents a RadioButton. */
    class RadioButton implements IRadioButton {

        /**
         * Constructs a new RadioButton.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRadioButton);

        /** RadioButton options. */
        public options: string[];

        /**
         * Creates a new RadioButton instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RadioButton instance
         */
        public static create(properties?: proto.IRadioButton): proto.RadioButton;

        /**
         * Encodes the specified RadioButton message. Does not implicitly {@link proto.RadioButton.verify|verify} messages.
         * @param message RadioButton message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRadioButton, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RadioButton message, length delimited. Does not implicitly {@link proto.RadioButton.verify|verify} messages.
         * @param message RadioButton message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRadioButton, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RadioButton message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RadioButton
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RadioButton;

        /**
         * Decodes a RadioButton message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RadioButton
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RadioButton;

        /**
         * Verifies a RadioButton message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RadioButton message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RadioButton
         */
        public static fromObject(object: { [k: string]: any }): proto.RadioButton;

        /**
         * Creates a plain object from a RadioButton message. Also converts values to other types if specified.
         * @param message RadioButton
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RadioButton, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RadioButton to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RadioButton
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a TextField. */
    interface ITextField {

        /** TextField text */
        text?: (string|null);
    }

    /** Represents a TextField. */
    class TextField implements ITextField {

        /**
         * Constructs a new TextField.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ITextField);

        /** TextField text. */
        public text: string;

        /**
         * Creates a new TextField instance using the specified properties.
         * @param [properties] Properties to set
         * @returns TextField instance
         */
        public static create(properties?: proto.ITextField): proto.TextField;

        /**
         * Encodes the specified TextField message. Does not implicitly {@link proto.TextField.verify|verify} messages.
         * @param message TextField message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ITextField, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified TextField message, length delimited. Does not implicitly {@link proto.TextField.verify|verify} messages.
         * @param message TextField message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ITextField, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a TextField message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns TextField
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.TextField;

        /**
         * Decodes a TextField message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns TextField
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.TextField;

        /**
         * Verifies a TextField message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a TextField message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns TextField
         */
        public static fromObject(object: { [k: string]: any }): proto.TextField;

        /**
         * Creates a plain object from a TextField message. Also converts values to other types if specified.
         * @param message TextField
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.TextField, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this TextField to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for TextField
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of an UiOptionDeclaration. */
    interface IUiOptionDeclaration {

        /** UiOptionDeclaration path */
        path?: (string|null);

        /** UiOptionDeclaration isMutable */
        isMutable?: (boolean|null);

        /** UiOptionDeclaration description */
        description?: (string|null);

        /** UiOptionDeclaration slider */
        slider?: (proto.ISlider|null);

        /** UiOptionDeclaration checkbox */
        checkbox?: (proto.ICheckbox|null);

        /** UiOptionDeclaration dropdown */
        dropdown?: (proto.IDropdown|null);

        /** UiOptionDeclaration radiobutton */
        radiobutton?: (proto.IRadioButton|null);

        /** UiOptionDeclaration textfield */
        textfield?: (proto.ITextField|null);

        /** UiOptionDeclaration default */
        "default"?: (proto.IUiValue|null);
    }

    /** Represents an UiOptionDeclaration. */
    class UiOptionDeclaration implements IUiOptionDeclaration {

        /**
         * Constructs a new UiOptionDeclaration.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IUiOptionDeclaration);

        /** UiOptionDeclaration path. */
        public path: string;

        /** UiOptionDeclaration isMutable. */
        public isMutable: boolean;

        /** UiOptionDeclaration description. */
        public description: string;

        /** UiOptionDeclaration slider. */
        public slider?: (proto.ISlider|null);

        /** UiOptionDeclaration checkbox. */
        public checkbox?: (proto.ICheckbox|null);

        /** UiOptionDeclaration dropdown. */
        public dropdown?: (proto.IDropdown|null);

        /** UiOptionDeclaration radiobutton. */
        public radiobutton?: (proto.IRadioButton|null);

        /** UiOptionDeclaration textfield. */
        public textfield?: (proto.ITextField|null);

        /** UiOptionDeclaration default. */
        public default?: (proto.IUiValue|null);

        /** UiOptionDeclaration uiElements. */
        public uiElements?: ("slider"|"checkbox"|"dropdown"|"radiobutton"|"textfield");

        /**
         * Creates a new UiOptionDeclaration instance using the specified properties.
         * @param [properties] Properties to set
         * @returns UiOptionDeclaration instance
         */
        public static create(properties?: proto.IUiOptionDeclaration): proto.UiOptionDeclaration;

        /**
         * Encodes the specified UiOptionDeclaration message. Does not implicitly {@link proto.UiOptionDeclaration.verify|verify} messages.
         * @param message UiOptionDeclaration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IUiOptionDeclaration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified UiOptionDeclaration message, length delimited. Does not implicitly {@link proto.UiOptionDeclaration.verify|verify} messages.
         * @param message UiOptionDeclaration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IUiOptionDeclaration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes an UiOptionDeclaration message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns UiOptionDeclaration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.UiOptionDeclaration;

        /**
         * Decodes an UiOptionDeclaration message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns UiOptionDeclaration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.UiOptionDeclaration;

        /**
         * Verifies an UiOptionDeclaration message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates an UiOptionDeclaration message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns UiOptionDeclaration
         */
        public static fromObject(object: { [k: string]: any }): proto.UiOptionDeclaration;

        /**
         * Creates a plain object from an UiOptionDeclaration message. Also converts values to other types if specified.
         * @param message UiOptionDeclaration
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.UiOptionDeclaration, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this UiOptionDeclaration to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for UiOptionDeclaration
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of an UiOptionDeclarations. */
    interface IUiOptionDeclarations {

        /** UiOptionDeclarations options */
        options?: (proto.IUiOptionDeclaration[]|null);
    }

    /** Represents an UiOptionDeclarations. */
    class UiOptionDeclarations implements IUiOptionDeclarations {

        /**
         * Constructs a new UiOptionDeclarations.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IUiOptionDeclarations);

        /** UiOptionDeclarations options. */
        public options: proto.IUiOptionDeclaration[];

        /**
         * Creates a new UiOptionDeclarations instance using the specified properties.
         * @param [properties] Properties to set
         * @returns UiOptionDeclarations instance
         */
        public static create(properties?: proto.IUiOptionDeclarations): proto.UiOptionDeclarations;

        /**
         * Encodes the specified UiOptionDeclarations message. Does not implicitly {@link proto.UiOptionDeclarations.verify|verify} messages.
         * @param message UiOptionDeclarations message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IUiOptionDeclarations, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified UiOptionDeclarations message, length delimited. Does not implicitly {@link proto.UiOptionDeclarations.verify|verify} messages.
         * @param message UiOptionDeclarations message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IUiOptionDeclarations, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes an UiOptionDeclarations message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns UiOptionDeclarations
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.UiOptionDeclarations;

        /**
         * Decodes an UiOptionDeclarations message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns UiOptionDeclarations
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.UiOptionDeclarations;

        /**
         * Verifies an UiOptionDeclarations message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates an UiOptionDeclarations message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns UiOptionDeclarations
         */
        public static fromObject(object: { [k: string]: any }): proto.UiOptionDeclarations;

        /**
         * Creates a plain object from an UiOptionDeclarations message. Also converts values to other types if specified.
         * @param message UiOptionDeclarations
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.UiOptionDeclarations, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this UiOptionDeclarations to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for UiOptionDeclarations
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of an UiValue. */
    interface IUiValue {

        /** UiValue floatValue */
        floatValue?: (number|null);

        /** UiValue boolValue */
        boolValue?: (boolean|null);

        /** UiValue integerValue */
        integerValue?: (number|Long|null);

        /** UiValue textValue */
        textValue?: (string|null);
    }

    /** Represents an UiValue. */
    class UiValue implements IUiValue {

        /**
         * Constructs a new UiValue.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IUiValue);

        /** UiValue floatValue. */
        public floatValue?: (number|null);

        /** UiValue boolValue. */
        public boolValue?: (boolean|null);

        /** UiValue integerValue. */
        public integerValue?: (number|Long|null);

        /** UiValue textValue. */
        public textValue?: (string|null);

        /** UiValue value. */
        public value?: ("floatValue"|"boolValue"|"integerValue"|"textValue");

        /**
         * Creates a new UiValue instance using the specified properties.
         * @param [properties] Properties to set
         * @returns UiValue instance
         */
        public static create(properties?: proto.IUiValue): proto.UiValue;

        /**
         * Encodes the specified UiValue message. Does not implicitly {@link proto.UiValue.verify|verify} messages.
         * @param message UiValue message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IUiValue, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified UiValue message, length delimited. Does not implicitly {@link proto.UiValue.verify|verify} messages.
         * @param message UiValue message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IUiValue, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes an UiValue message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns UiValue
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.UiValue;

        /**
         * Decodes an UiValue message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns UiValue
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.UiValue;

        /**
         * Verifies an UiValue message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates an UiValue message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns UiValue
         */
        public static fromObject(object: { [k: string]: any }): proto.UiValue;

        /**
         * Creates a plain object from an UiValue message. Also converts values to other types if specified.
         * @param message UiValue
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.UiValue, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this UiValue to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for UiValue
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of an UiValues. */
    interface IUiValues {

        /** UiValues uiValues */
        uiValues?: ({ [k: string]: proto.IUiValue }|null);
    }

    /** Represents an UiValues. */
    class UiValues implements IUiValues {

        /**
         * Constructs a new UiValues.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IUiValues);

        /** UiValues uiValues. */
        public uiValues: { [k: string]: proto.IUiValue };

        /**
         * Creates a new UiValues instance using the specified properties.
         * @param [properties] Properties to set
         * @returns UiValues instance
         */
        public static create(properties?: proto.IUiValues): proto.UiValues;

        /**
         * Encodes the specified UiValues message. Does not implicitly {@link proto.UiValues.verify|verify} messages.
         * @param message UiValues message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IUiValues, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified UiValues message, length delimited. Does not implicitly {@link proto.UiValues.verify|verify} messages.
         * @param message UiValues message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IUiValues, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes an UiValues message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns UiValues
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.UiValues;

        /**
         * Decodes an UiValues message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns UiValues
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.UiValues;

        /**
         * Verifies an UiValues message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates an UiValues message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns UiValues
         */
        public static fromObject(object: { [k: string]: any }): proto.UiValues;

        /**
         * Creates a plain object from an UiValues message. Also converts values to other types if specified.
         * @param message UiValues
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.UiValues, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this UiValues to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for UiValues
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Drawing. */
    interface IDrawing {

        /** Drawing retainForTicks */
        retainForTicks?: (number|null);

        /** Drawing label */
        label?: (string|null);

        /** Drawing color */
        color?: (proto.Drawing.Color|null);

        /** Drawing method */
        method?: (proto.Drawing.Method|null);

        /** Drawing points */
        points?: (proto.IVector2f[]|null);

        /** Drawing category */
        category?: (proto.Drawing.Category|null);

        /** Drawing forRobotId */
        forRobotId?: (number|null);
    }

    /** Represents a Drawing. */
    class Drawing implements IDrawing {

        /**
         * Constructs a new Drawing.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IDrawing);

        /** Drawing retainForTicks. */
        public retainForTicks: number;

        /** Drawing label. */
        public label: string;

        /** Drawing color. */
        public color: proto.Drawing.Color;

        /** Drawing method. */
        public method: proto.Drawing.Method;

        /** Drawing points. */
        public points: proto.IVector2f[];

        /** Drawing category. */
        public category: proto.Drawing.Category;

        /** Drawing forRobotId. */
        public forRobotId: number;

        /**
         * Creates a new Drawing instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Drawing instance
         */
        public static create(properties?: proto.IDrawing): proto.Drawing;

        /**
         * Encodes the specified Drawing message. Does not implicitly {@link proto.Drawing.verify|verify} messages.
         * @param message Drawing message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IDrawing, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Drawing message, length delimited. Does not implicitly {@link proto.Drawing.verify|verify} messages.
         * @param message Drawing message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IDrawing, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Drawing message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Drawing
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Drawing;

        /**
         * Decodes a Drawing message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Drawing
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Drawing;

        /**
         * Verifies a Drawing message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Drawing message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Drawing
         */
        public static fromObject(object: { [k: string]: any }): proto.Drawing;

        /**
         * Creates a plain object from a Drawing message. Also converts values to other types if specified.
         * @param message Drawing
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Drawing, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Drawing to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Drawing
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace Drawing {

        /** Method enum. */
        enum Method {
            LINES_CONNECTED = 0,
            DOTS = 1,
            CROSSES = 2,
            PLUSES = 3
        }

        /** Color enum. */
        enum Color {
            RED = 0,
            GREEN = 1,
            BLUE = 2,
            YELLOW = 3,
            CYAN = 4,
            MAGENTA = 5,
            WHITE = 6,
            BLACK = 7
        }

        /** Category enum. */
        enum Category {
            PATH_PLANNING = 0,
            DEBUG = 1
        }
    }

    /** Properties of a Metric. */
    interface IMetric {

        /** Metric label */
        label?: (string|null);

        /** Metric boundedValue */
        boundedValue?: (proto.Metric.IBoundedValue|null);

        /** Metric decimal */
        decimal?: (proto.Metric.IDecimal|null);
    }

    /** Represents a Metric. */
    class Metric implements IMetric {

        /**
         * Constructs a new Metric.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IMetric);

        /** Metric label. */
        public label: string;

        /** Metric boundedValue. */
        public boundedValue?: (proto.Metric.IBoundedValue|null);

        /** Metric decimal. */
        public decimal?: (proto.Metric.IDecimal|null);

        /** Metric value. */
        public value?: ("boundedValue"|"decimal");

        /**
         * Creates a new Metric instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Metric instance
         */
        public static create(properties?: proto.IMetric): proto.Metric;

        /**
         * Encodes the specified Metric message. Does not implicitly {@link proto.Metric.verify|verify} messages.
         * @param message Metric message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IMetric, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Metric message, length delimited. Does not implicitly {@link proto.Metric.verify|verify} messages.
         * @param message Metric message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IMetric, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Metric message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Metric
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Metric;

        /**
         * Decodes a Metric message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Metric
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Metric;

        /**
         * Verifies a Metric message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Metric message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Metric
         */
        public static fromObject(object: { [k: string]: any }): proto.Metric;

        /**
         * Creates a plain object from a Metric message. Also converts values to other types if specified.
         * @param message Metric
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Metric, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Metric to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Metric
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace Metric {

        /** Properties of a BoundedValue. */
        interface IBoundedValue {

            /** BoundedValue value */
            value?: (number|null);

            /** BoundedValue min */
            min?: (number|null);

            /** BoundedValue max */
            max?: (number|null);

            /** BoundedValue unit */
            unit?: (string|null);
        }

        /** Represents a BoundedValue. */
        class BoundedValue implements IBoundedValue {

            /**
             * Constructs a new BoundedValue.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.Metric.IBoundedValue);

            /** BoundedValue value. */
            public value: number;

            /** BoundedValue min. */
            public min: number;

            /** BoundedValue max. */
            public max: number;

            /** BoundedValue unit. */
            public unit: string;

            /**
             * Creates a new BoundedValue instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BoundedValue instance
             */
            public static create(properties?: proto.Metric.IBoundedValue): proto.Metric.BoundedValue;

            /**
             * Encodes the specified BoundedValue message. Does not implicitly {@link proto.Metric.BoundedValue.verify|verify} messages.
             * @param message BoundedValue message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.Metric.IBoundedValue, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BoundedValue message, length delimited. Does not implicitly {@link proto.Metric.BoundedValue.verify|verify} messages.
             * @param message BoundedValue message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.Metric.IBoundedValue, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BoundedValue message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BoundedValue
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Metric.BoundedValue;

            /**
             * Decodes a BoundedValue message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BoundedValue
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Metric.BoundedValue;

            /**
             * Verifies a BoundedValue message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BoundedValue message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BoundedValue
             */
            public static fromObject(object: { [k: string]: any }): proto.Metric.BoundedValue;

            /**
             * Creates a plain object from a BoundedValue message. Also converts values to other types if specified.
             * @param message BoundedValue
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.Metric.BoundedValue, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BoundedValue to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BoundedValue
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Decimal. */
        interface IDecimal {

            /** Decimal value */
            value?: (number|null);

            /** Decimal minRecorded */
            minRecorded?: (number|null);

            /** Decimal maxRecorded */
            maxRecorded?: (number|null);

            /** Decimal unit */
            unit?: (string|null);
        }

        /** Represents a Decimal. */
        class Decimal implements IDecimal {

            /**
             * Constructs a new Decimal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.Metric.IDecimal);

            /** Decimal value. */
            public value: number;

            /** Decimal minRecorded. */
            public minRecorded: number;

            /** Decimal maxRecorded. */
            public maxRecorded: number;

            /** Decimal unit. */
            public unit: string;

            /**
             * Creates a new Decimal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Decimal instance
             */
            public static create(properties?: proto.Metric.IDecimal): proto.Metric.Decimal;

            /**
             * Encodes the specified Decimal message. Does not implicitly {@link proto.Metric.Decimal.verify|verify} messages.
             * @param message Decimal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.Metric.IDecimal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Decimal message, length delimited. Does not implicitly {@link proto.Metric.Decimal.verify|verify} messages.
             * @param message Decimal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.Metric.IDecimal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Decimal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Decimal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Metric.Decimal;

            /**
             * Decodes a Decimal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Decimal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Metric.Decimal;

            /**
             * Verifies a Decimal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Decimal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Decimal
             */
            public static fromObject(object: { [k: string]: any }): proto.Metric.Decimal;

            /**
             * Creates a plain object from a Decimal message. Also converts values to other types if specified.
             * @param message Decimal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.Metric.Decimal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Decimal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Decimal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a STPStatus. */
    interface ISTPStatus {

        /** STPStatus selectedPlay */
        selectedPlay?: (proto.STPStatus.IScoredPlay|null);

        /** STPStatus robots */
        robots?: ({ [k: string]: proto.STPStatus.ISTPRobot }|null);

        /** STPStatus scoredPlays */
        scoredPlays?: (proto.STPStatus.IScoredPlay[]|null);

        /** STPStatus currentTick */
        currentTick?: (number|null);
    }

    /** Represents a STPStatus. */
    class STPStatus implements ISTPStatus {

        /**
         * Constructs a new STPStatus.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISTPStatus);

        /** STPStatus selectedPlay. */
        public selectedPlay?: (proto.STPStatus.IScoredPlay|null);

        /** STPStatus robots. */
        public robots: { [k: string]: proto.STPStatus.ISTPRobot };

        /** STPStatus scoredPlays. */
        public scoredPlays: proto.STPStatus.IScoredPlay[];

        /** STPStatus currentTick. */
        public currentTick: number;

        /**
         * Creates a new STPStatus instance using the specified properties.
         * @param [properties] Properties to set
         * @returns STPStatus instance
         */
        public static create(properties?: proto.ISTPStatus): proto.STPStatus;

        /**
         * Encodes the specified STPStatus message. Does not implicitly {@link proto.STPStatus.verify|verify} messages.
         * @param message STPStatus message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISTPStatus, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified STPStatus message, length delimited. Does not implicitly {@link proto.STPStatus.verify|verify} messages.
         * @param message STPStatus message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISTPStatus, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a STPStatus message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns STPStatus
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus;

        /**
         * Decodes a STPStatus message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns STPStatus
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus;

        /**
         * Verifies a STPStatus message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a STPStatus message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns STPStatus
         */
        public static fromObject(object: { [k: string]: any }): proto.STPStatus;

        /**
         * Creates a plain object from a STPStatus message. Also converts values to other types if specified.
         * @param message STPStatus
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.STPStatus, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this STPStatus to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for STPStatus
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace STPStatus {

        /** Properties of a STPRobot. */
        interface ISTPRobot {

            /** STPRobot id */
            id?: (number|null);

            /** STPRobot role */
            role?: (proto.STPStatus.STPRobot.IRole|null);

            /** STPRobot tactic */
            tactic?: (proto.STPStatus.STPRobot.ITactic|null);

            /** STPRobot skill */
            skill?: (proto.STPStatus.STPRobot.ISkill|null);
        }

        /** Represents a STPRobot. */
        class STPRobot implements ISTPRobot {

            /**
             * Constructs a new STPRobot.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.STPStatus.ISTPRobot);

            /** STPRobot id. */
            public id: number;

            /** STPRobot role. */
            public role?: (proto.STPStatus.STPRobot.IRole|null);

            /** STPRobot tactic. */
            public tactic?: (proto.STPStatus.STPRobot.ITactic|null);

            /** STPRobot skill. */
            public skill?: (proto.STPStatus.STPRobot.ISkill|null);

            /**
             * Creates a new STPRobot instance using the specified properties.
             * @param [properties] Properties to set
             * @returns STPRobot instance
             */
            public static create(properties?: proto.STPStatus.ISTPRobot): proto.STPStatus.STPRobot;

            /**
             * Encodes the specified STPRobot message. Does not implicitly {@link proto.STPStatus.STPRobot.verify|verify} messages.
             * @param message STPRobot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.STPStatus.ISTPRobot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified STPRobot message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.verify|verify} messages.
             * @param message STPRobot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.STPStatus.ISTPRobot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a STPRobot message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns STPRobot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot;

            /**
             * Decodes a STPRobot message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns STPRobot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot;

            /**
             * Verifies a STPRobot message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a STPRobot message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns STPRobot
             */
            public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot;

            /**
             * Creates a plain object from a STPRobot message. Also converts values to other types if specified.
             * @param message STPRobot
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.STPStatus.STPRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this STPRobot to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for STPRobot
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        namespace STPRobot {

            /** Status enum. */
            enum Status {
                WAITING = 0,
                SUCCESSFUL = 1,
                FAILURE = 2,
                RUNNING = 3
            }

            /** Properties of a Role. */
            interface IRole {

                /** Role name */
                name?: (string|null);

                /** Role status */
                status?: (proto.STPStatus.STPRobot.Status|null);
            }

            /** Represents a Role. */
            class Role implements IRole {

                /**
                 * Constructs a new Role.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.STPStatus.STPRobot.IRole);

                /** Role name. */
                public name: string;

                /** Role status. */
                public status: proto.STPStatus.STPRobot.Status;

                /**
                 * Creates a new Role instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns Role instance
                 */
                public static create(properties?: proto.STPStatus.STPRobot.IRole): proto.STPStatus.STPRobot.Role;

                /**
                 * Encodes the specified Role message. Does not implicitly {@link proto.STPStatus.STPRobot.Role.verify|verify} messages.
                 * @param message Role message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.STPStatus.STPRobot.IRole, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified Role message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.Role.verify|verify} messages.
                 * @param message Role message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.STPStatus.STPRobot.IRole, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Role message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Role
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot.Role;

                /**
                 * Decodes a Role message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns Role
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot.Role;

                /**
                 * Verifies a Role message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a Role message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns Role
                 */
                public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot.Role;

                /**
                 * Creates a plain object from a Role message. Also converts values to other types if specified.
                 * @param message Role
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.STPStatus.STPRobot.Role, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this Role to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for Role
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }

            /** Properties of a Tactic. */
            interface ITactic {

                /** Tactic name */
                name?: (string|null);

                /** Tactic status */
                status?: (proto.STPStatus.STPRobot.Status|null);
            }

            /** Represents a Tactic. */
            class Tactic implements ITactic {

                /**
                 * Constructs a new Tactic.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.STPStatus.STPRobot.ITactic);

                /** Tactic name. */
                public name: string;

                /** Tactic status. */
                public status: proto.STPStatus.STPRobot.Status;

                /**
                 * Creates a new Tactic instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns Tactic instance
                 */
                public static create(properties?: proto.STPStatus.STPRobot.ITactic): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Encodes the specified Tactic message. Does not implicitly {@link proto.STPStatus.STPRobot.Tactic.verify|verify} messages.
                 * @param message Tactic message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.STPStatus.STPRobot.ITactic, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified Tactic message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.Tactic.verify|verify} messages.
                 * @param message Tactic message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.STPStatus.STPRobot.ITactic, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Tactic message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Tactic
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Decodes a Tactic message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns Tactic
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Verifies a Tactic message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a Tactic message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns Tactic
                 */
                public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Creates a plain object from a Tactic message. Also converts values to other types if specified.
                 * @param message Tactic
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.STPStatus.STPRobot.Tactic, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this Tactic to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for Tactic
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }

            /** Properties of a Skill. */
            interface ISkill {

                /** Skill name */
                name?: (string|null);

                /** Skill status */
                status?: (proto.STPStatus.STPRobot.Status|null);
            }

            /** Represents a Skill. */
            class Skill implements ISkill {

                /**
                 * Constructs a new Skill.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.STPStatus.STPRobot.ISkill);

                /** Skill name. */
                public name: string;

                /** Skill status. */
                public status: proto.STPStatus.STPRobot.Status;

                /**
                 * Creates a new Skill instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns Skill instance
                 */
                public static create(properties?: proto.STPStatus.STPRobot.ISkill): proto.STPStatus.STPRobot.Skill;

                /**
                 * Encodes the specified Skill message. Does not implicitly {@link proto.STPStatus.STPRobot.Skill.verify|verify} messages.
                 * @param message Skill message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.STPStatus.STPRobot.ISkill, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified Skill message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.Skill.verify|verify} messages.
                 * @param message Skill message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.STPStatus.STPRobot.ISkill, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Skill message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Skill
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot.Skill;

                /**
                 * Decodes a Skill message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns Skill
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot.Skill;

                /**
                 * Verifies a Skill message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a Skill message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns Skill
                 */
                public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot.Skill;

                /**
                 * Creates a plain object from a Skill message. Also converts values to other types if specified.
                 * @param message Skill
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.STPStatus.STPRobot.Skill, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this Skill to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for Skill
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }
        }

        /** Properties of a ScoredPlay. */
        interface IScoredPlay {

            /** ScoredPlay playName */
            playName?: (string|null);

            /** ScoredPlay playScore */
            playScore?: (number|null);
        }

        /** Represents a ScoredPlay. */
        class ScoredPlay implements IScoredPlay {

            /**
             * Constructs a new ScoredPlay.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.STPStatus.IScoredPlay);

            /** ScoredPlay playName. */
            public playName: string;

            /** ScoredPlay playScore. */
            public playScore: number;

            /**
             * Creates a new ScoredPlay instance using the specified properties.
             * @param [properties] Properties to set
             * @returns ScoredPlay instance
             */
            public static create(properties?: proto.STPStatus.IScoredPlay): proto.STPStatus.ScoredPlay;

            /**
             * Encodes the specified ScoredPlay message. Does not implicitly {@link proto.STPStatus.ScoredPlay.verify|verify} messages.
             * @param message ScoredPlay message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.STPStatus.IScoredPlay, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified ScoredPlay message, length delimited. Does not implicitly {@link proto.STPStatus.ScoredPlay.verify|verify} messages.
             * @param message ScoredPlay message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.STPStatus.IScoredPlay, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ScoredPlay message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ScoredPlay
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.ScoredPlay;

            /**
             * Decodes a ScoredPlay message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns ScoredPlay
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.ScoredPlay;

            /**
             * Verifies a ScoredPlay message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a ScoredPlay message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns ScoredPlay
             */
            public static fromObject(object: { [k: string]: any }): proto.STPStatus.ScoredPlay;

            /**
             * Creates a plain object from a ScoredPlay message. Also converts values to other types if specified.
             * @param message ScoredPlay
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.STPStatus.ScoredPlay, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this ScoredPlay to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for ScoredPlay
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a SetupMessage. */
    interface ISetupMessage {

        /** SetupMessage availablePlays */
        availablePlays?: (string[]|null);

        /** SetupMessage availableRulesets */
        availableRulesets?: (string[]|null);

        /** SetupMessage isPaused */
        isPaused?: (boolean|null);

        /** SetupMessage gameSettings */
        gameSettings?: (proto.IGameSettings|null);

        /** SetupMessage aiSettings */
        aiSettings?: (proto.IAISettings|null);
    }

    /** Represents a SetupMessage. */
    class SetupMessage implements ISetupMessage {

        /**
         * Constructs a new SetupMessage.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISetupMessage);

        /** SetupMessage availablePlays. */
        public availablePlays: string[];

        /** SetupMessage availableRulesets. */
        public availableRulesets: string[];

        /** SetupMessage isPaused. */
        public isPaused: boolean;

        /** SetupMessage gameSettings. */
        public gameSettings?: (proto.IGameSettings|null);

        /** SetupMessage aiSettings. */
        public aiSettings?: (proto.IAISettings|null);

        /**
         * Creates a new SetupMessage instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SetupMessage instance
         */
        public static create(properties?: proto.ISetupMessage): proto.SetupMessage;

        /**
         * Encodes the specified SetupMessage message. Does not implicitly {@link proto.SetupMessage.verify|verify} messages.
         * @param message SetupMessage message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISetupMessage, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SetupMessage message, length delimited. Does not implicitly {@link proto.SetupMessage.verify|verify} messages.
         * @param message SetupMessage message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISetupMessage, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SetupMessage message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SetupMessage
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SetupMessage;

        /**
         * Decodes a SetupMessage message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SetupMessage
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SetupMessage;

        /**
         * Verifies a SetupMessage message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SetupMessage message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SetupMessage
         */
        public static fromObject(object: { [k: string]: any }): proto.SetupMessage;

        /**
         * Creates a plain object from a SetupMessage message. Also converts values to other types if specified.
         * @param message SetupMessage
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SetupMessage, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SetupMessage to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SetupMessage
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a GameState. */
    interface IGameState {

        /** GameState playName */
        playName?: (string|null);

        /** GameState rulesetName */
        rulesetName?: (string|null);

        /** GameState keeperId */
        keeperId?: (number|null);
    }

    /** Represents a GameState. */
    class GameState implements IGameState {

        /**
         * Constructs a new GameState.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IGameState);

        /** GameState playName. */
        public playName: string;

        /** GameState rulesetName. */
        public rulesetName: string;

        /** GameState keeperId. */
        public keeperId: number;

        /**
         * Creates a new GameState instance using the specified properties.
         * @param [properties] Properties to set
         * @returns GameState instance
         */
        public static create(properties?: proto.IGameState): proto.GameState;

        /**
         * Encodes the specified GameState message. Does not implicitly {@link proto.GameState.verify|verify} messages.
         * @param message GameState message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IGameState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified GameState message, length delimited. Does not implicitly {@link proto.GameState.verify|verify} messages.
         * @param message GameState message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IGameState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a GameState message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns GameState
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameState;

        /**
         * Decodes a GameState message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns GameState
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameState;

        /**
         * Verifies a GameState message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a GameState message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns GameState
         */
        public static fromObject(object: { [k: string]: any }): proto.GameState;

        /**
         * Creates a plain object from a GameState message. Also converts values to other types if specified.
         * @param message GameState
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.GameState, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this GameState to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for GameState
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a AISettings. */
    interface IAISettings {

        /** AISettings useReferee */
        useReferee?: (boolean|null);

        /** AISettings ignoreInvariants */
        ignoreInvariants?: (boolean|null);
    }

    /** Represents a AISettings. */
    class AISettings implements IAISettings {

        /**
         * Constructs a new AISettings.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IAISettings);

        /** AISettings useReferee. */
        public useReferee: boolean;

        /** AISettings ignoreInvariants. */
        public ignoreInvariants: boolean;

        /**
         * Creates a new AISettings instance using the specified properties.
         * @param [properties] Properties to set
         * @returns AISettings instance
         */
        public static create(properties?: proto.IAISettings): proto.AISettings;

        /**
         * Encodes the specified AISettings message. Does not implicitly {@link proto.AISettings.verify|verify} messages.
         * @param message AISettings message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IAISettings, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified AISettings message, length delimited. Does not implicitly {@link proto.AISettings.verify|verify} messages.
         * @param message AISettings message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IAISettings, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a AISettings message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns AISettings
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.AISettings;

        /**
         * Decodes a AISettings message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns AISettings
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.AISettings;

        /**
         * Verifies a AISettings message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a AISettings message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns AISettings
         */
        public static fromObject(object: { [k: string]: any }): proto.AISettings;

        /**
         * Creates a plain object from a AISettings message. Also converts values to other types if specified.
         * @param message AISettings
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.AISettings, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this AISettings to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for AISettings
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a MsgToInterface. */
    interface IMsgToInterface {

        /** MsgToInterface stpStatus */
        stpStatus?: (proto.ISTPStatus|null);

        /** MsgToInterface setupMessage */
        setupMessage?: (proto.ISetupMessage|null);

        /** MsgToInterface state */
        state?: (proto.IState|null);

        /** MsgToInterface visualizations */
        visualizations?: (proto.MsgToInterface.IVisualizationBuffer|null);
    }

    /** Represents a MsgToInterface. */
    class MsgToInterface implements IMsgToInterface {

        /**
         * Constructs a new MsgToInterface.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IMsgToInterface);

        /** MsgToInterface stpStatus. */
        public stpStatus?: (proto.ISTPStatus|null);

        /** MsgToInterface setupMessage. */
        public setupMessage?: (proto.ISetupMessage|null);

        /** MsgToInterface state. */
        public state?: (proto.IState|null);

        /** MsgToInterface visualizations. */
        public visualizations?: (proto.MsgToInterface.IVisualizationBuffer|null);

        /** MsgToInterface kind. */
        public kind?: ("stpStatus"|"setupMessage"|"state"|"visualizations");

        /**
         * Creates a new MsgToInterface instance using the specified properties.
         * @param [properties] Properties to set
         * @returns MsgToInterface instance
         */
        public static create(properties?: proto.IMsgToInterface): proto.MsgToInterface;

        /**
         * Encodes the specified MsgToInterface message. Does not implicitly {@link proto.MsgToInterface.verify|verify} messages.
         * @param message MsgToInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IMsgToInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified MsgToInterface message, length delimited. Does not implicitly {@link proto.MsgToInterface.verify|verify} messages.
         * @param message MsgToInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IMsgToInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a MsgToInterface message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns MsgToInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.MsgToInterface;

        /**
         * Decodes a MsgToInterface message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns MsgToInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.MsgToInterface;

        /**
         * Verifies a MsgToInterface message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a MsgToInterface message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns MsgToInterface
         */
        public static fromObject(object: { [k: string]: any }): proto.MsgToInterface;

        /**
         * Creates a plain object from a MsgToInterface message. Also converts values to other types if specified.
         * @param message MsgToInterface
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.MsgToInterface, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this MsgToInterface to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for MsgToInterface
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace MsgToInterface {

        /** Properties of a VisualizationBuffer. */
        interface IVisualizationBuffer {

            /** VisualizationBuffer drawings */
            drawings?: (proto.IDrawing[]|null);

            /** VisualizationBuffer metrics */
            metrics?: (proto.IMetric[]|null);
        }

        /** Represents a VisualizationBuffer. */
        class VisualizationBuffer implements IVisualizationBuffer {

            /**
             * Constructs a new VisualizationBuffer.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.MsgToInterface.IVisualizationBuffer);

            /** VisualizationBuffer drawings. */
            public drawings: proto.IDrawing[];

            /** VisualizationBuffer metrics. */
            public metrics: proto.IMetric[];

            /**
             * Creates a new VisualizationBuffer instance using the specified properties.
             * @param [properties] Properties to set
             * @returns VisualizationBuffer instance
             */
            public static create(properties?: proto.MsgToInterface.IVisualizationBuffer): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Encodes the specified VisualizationBuffer message. Does not implicitly {@link proto.MsgToInterface.VisualizationBuffer.verify|verify} messages.
             * @param message VisualizationBuffer message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.MsgToInterface.IVisualizationBuffer, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified VisualizationBuffer message, length delimited. Does not implicitly {@link proto.MsgToInterface.VisualizationBuffer.verify|verify} messages.
             * @param message VisualizationBuffer message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.MsgToInterface.IVisualizationBuffer, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a VisualizationBuffer message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns VisualizationBuffer
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Decodes a VisualizationBuffer message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns VisualizationBuffer
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Verifies a VisualizationBuffer message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a VisualizationBuffer message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns VisualizationBuffer
             */
            public static fromObject(object: { [k: string]: any }): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Creates a plain object from a VisualizationBuffer message. Also converts values to other types if specified.
             * @param message VisualizationBuffer
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.MsgToInterface.VisualizationBuffer, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this VisualizationBuffer to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for VisualizationBuffer
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a MsgFromInterface. */
    interface IMsgFromInterface {

        /** MsgFromInterface setGameState */
        setGameState?: (proto.IGameState|null);

        /** MsgFromInterface setGameSettings */
        setGameSettings?: (proto.IGameSettings|null);

        /** MsgFromInterface stopResume */
        stopResume?: (boolean|null);

        /** MsgFromInterface setAiSettings */
        setAiSettings?: (proto.IAISettings|null);
    }

    /** Represents a MsgFromInterface. */
    class MsgFromInterface implements IMsgFromInterface {

        /**
         * Constructs a new MsgFromInterface.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IMsgFromInterface);

        /** MsgFromInterface setGameState. */
        public setGameState?: (proto.IGameState|null);

        /** MsgFromInterface setGameSettings. */
        public setGameSettings?: (proto.IGameSettings|null);

        /** MsgFromInterface stopResume. */
        public stopResume?: (boolean|null);

        /** MsgFromInterface setAiSettings. */
        public setAiSettings?: (proto.IAISettings|null);

        /** MsgFromInterface kind. */
        public kind?: ("setGameState"|"setGameSettings"|"stopResume"|"setAiSettings");

        /**
         * Creates a new MsgFromInterface instance using the specified properties.
         * @param [properties] Properties to set
         * @returns MsgFromInterface instance
         */
        public static create(properties?: proto.IMsgFromInterface): proto.MsgFromInterface;

        /**
         * Encodes the specified MsgFromInterface message. Does not implicitly {@link proto.MsgFromInterface.verify|verify} messages.
         * @param message MsgFromInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IMsgFromInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified MsgFromInterface message, length delimited. Does not implicitly {@link proto.MsgFromInterface.verify|verify} messages.
         * @param message MsgFromInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IMsgFromInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a MsgFromInterface message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns MsgFromInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.MsgFromInterface;

        /**
         * Decodes a MsgFromInterface message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns MsgFromInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.MsgFromInterface;

        /**
         * Verifies a MsgFromInterface message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a MsgFromInterface message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns MsgFromInterface
         */
        public static fromObject(object: { [k: string]: any }): proto.MsgFromInterface;

        /**
         * Creates a plain object from a MsgFromInterface message. Also converts values to other types if specified.
         * @param message MsgFromInterface
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.MsgFromInterface, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this MsgFromInterface to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for MsgFromInterface
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a World. */
    interface IWorld {

        /** World time */
        time?: (number|Long|null);

        /** World id */
        id?: (number|null);

        /** World ball */
        ball?: (proto.IWorldBall|null);

        /** World yellow */
        yellow?: (proto.IWorldRobot[]|null);

        /** World blue */
        blue?: (proto.IWorldRobot[]|null);

        /** World yellowUnseenRobots */
        yellowUnseenRobots?: (proto.IFeedbackOnlyRobot[]|null);

        /** World blueUnseenRobots */
        blueUnseenRobots?: (proto.IFeedbackOnlyRobot[]|null);
    }

    /** Represents a World. */
    class World implements IWorld {

        /**
         * Constructs a new World.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IWorld);

        /** World time. */
        public time: (number|Long);

        /** World id. */
        public id: number;

        /** World ball. */
        public ball?: (proto.IWorldBall|null);

        /** World yellow. */
        public yellow: proto.IWorldRobot[];

        /** World blue. */
        public blue: proto.IWorldRobot[];

        /** World yellowUnseenRobots. */
        public yellowUnseenRobots: proto.IFeedbackOnlyRobot[];

        /** World blueUnseenRobots. */
        public blueUnseenRobots: proto.IFeedbackOnlyRobot[];

        /**
         * Creates a new World instance using the specified properties.
         * @param [properties] Properties to set
         * @returns World instance
         */
        public static create(properties?: proto.IWorld): proto.World;

        /**
         * Encodes the specified World message. Does not implicitly {@link proto.World.verify|verify} messages.
         * @param message World message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IWorld, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified World message, length delimited. Does not implicitly {@link proto.World.verify|verify} messages.
         * @param message World message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IWorld, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a World message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns World
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.World;

        /**
         * Decodes a World message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns World
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.World;

        /**
         * Verifies a World message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a World message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns World
         */
        public static fromObject(object: { [k: string]: any }): proto.World;

        /**
         * Creates a plain object from a World message. Also converts values to other types if specified.
         * @param message World
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.World, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this World to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for World
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a WorldBall. */
    interface IWorldBall {

        /** WorldBall area */
        area?: (number|null);

        /** WorldBall pos */
        pos?: (proto.IVector2f|null);

        /** WorldBall z */
        z?: (number|null);

        /** WorldBall vel */
        vel?: (proto.IVector2f|null);

        /** WorldBall zVel */
        zVel?: (number|null);

        /** WorldBall visible */
        visible?: (boolean|null);
    }

    /** Represents a WorldBall. */
    class WorldBall implements IWorldBall {

        /**
         * Constructs a new WorldBall.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IWorldBall);

        /** WorldBall area. */
        public area: number;

        /** WorldBall pos. */
        public pos?: (proto.IVector2f|null);

        /** WorldBall z. */
        public z: number;

        /** WorldBall vel. */
        public vel?: (proto.IVector2f|null);

        /** WorldBall zVel. */
        public zVel: number;

        /** WorldBall visible. */
        public visible: boolean;

        /**
         * Creates a new WorldBall instance using the specified properties.
         * @param [properties] Properties to set
         * @returns WorldBall instance
         */
        public static create(properties?: proto.IWorldBall): proto.WorldBall;

        /**
         * Encodes the specified WorldBall message. Does not implicitly {@link proto.WorldBall.verify|verify} messages.
         * @param message WorldBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IWorldBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified WorldBall message, length delimited. Does not implicitly {@link proto.WorldBall.verify|verify} messages.
         * @param message WorldBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IWorldBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a WorldBall message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns WorldBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.WorldBall;

        /**
         * Decodes a WorldBall message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns WorldBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.WorldBall;

        /**
         * Verifies a WorldBall message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a WorldBall message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns WorldBall
         */
        public static fromObject(object: { [k: string]: any }): proto.WorldBall;

        /**
         * Creates a plain object from a WorldBall message. Also converts values to other types if specified.
         * @param message WorldBall
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.WorldBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this WorldBall to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for WorldBall
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Vector2f. */
    interface IVector2f {

        /** Vector2f x */
        x?: (number|null);

        /** Vector2f y */
        y?: (number|null);
    }

    /** Represents a Vector2f. */
    class Vector2f implements IVector2f {

        /**
         * Constructs a new Vector2f.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IVector2f);

        /** Vector2f x. */
        public x: number;

        /** Vector2f y. */
        public y: number;

        /**
         * Creates a new Vector2f instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Vector2f instance
         */
        public static create(properties?: proto.IVector2f): proto.Vector2f;

        /**
         * Encodes the specified Vector2f message. Does not implicitly {@link proto.Vector2f.verify|verify} messages.
         * @param message Vector2f message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IVector2f, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Vector2f message, length delimited. Does not implicitly {@link proto.Vector2f.verify|verify} messages.
         * @param message Vector2f message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IVector2f, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Vector2f message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Vector2f
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Vector2f;

        /**
         * Decodes a Vector2f message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Vector2f
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Vector2f;

        /**
         * Verifies a Vector2f message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Vector2f message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Vector2f
         */
        public static fromObject(object: { [k: string]: any }): proto.Vector2f;

        /**
         * Creates a plain object from a Vector2f message. Also converts values to other types if specified.
         * @param message Vector2f
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Vector2f, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Vector2f to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Vector2f
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a WorldRobot. */
    interface IWorldRobot {

        /** WorldRobot id */
        id?: (number|null);

        /** WorldRobot pos */
        pos?: (proto.IVector2f|null);

        /** WorldRobot angle */
        angle?: (number|null);

        /** WorldRobot vel */
        vel?: (proto.IVector2f|null);

        /** WorldRobot w */
        w?: (number|null);

        /** WorldRobot feedbackInfo */
        feedbackInfo?: (proto.IRobotProcessedFeedback|null);
    }

    /** Represents a WorldRobot. */
    class WorldRobot implements IWorldRobot {

        /**
         * Constructs a new WorldRobot.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IWorldRobot);

        /** WorldRobot id. */
        public id: number;

        /** WorldRobot pos. */
        public pos?: (proto.IVector2f|null);

        /** WorldRobot angle. */
        public angle: number;

        /** WorldRobot vel. */
        public vel?: (proto.IVector2f|null);

        /** WorldRobot w. */
        public w: number;

        /** WorldRobot feedbackInfo. */
        public feedbackInfo?: (proto.IRobotProcessedFeedback|null);

        /**
         * Creates a new WorldRobot instance using the specified properties.
         * @param [properties] Properties to set
         * @returns WorldRobot instance
         */
        public static create(properties?: proto.IWorldRobot): proto.WorldRobot;

        /**
         * Encodes the specified WorldRobot message. Does not implicitly {@link proto.WorldRobot.verify|verify} messages.
         * @param message WorldRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IWorldRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified WorldRobot message, length delimited. Does not implicitly {@link proto.WorldRobot.verify|verify} messages.
         * @param message WorldRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IWorldRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a WorldRobot message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns WorldRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.WorldRobot;

        /**
         * Decodes a WorldRobot message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns WorldRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.WorldRobot;

        /**
         * Verifies a WorldRobot message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a WorldRobot message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns WorldRobot
         */
        public static fromObject(object: { [k: string]: any }): proto.WorldRobot;

        /**
         * Creates a plain object from a WorldRobot message. Also converts values to other types if specified.
         * @param message WorldRobot
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.WorldRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this WorldRobot to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for WorldRobot
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotWheel. */
    interface IRobotWheel {

        /** RobotWheel locked */
        locked?: (boolean|null);

        /** RobotWheel braking */
        braking?: (boolean|null);
    }

    /** Represents a RobotWheel. */
    class RobotWheel implements IRobotWheel {

        /**
         * Constructs a new RobotWheel.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotWheel);

        /** RobotWheel locked. */
        public locked: boolean;

        /** RobotWheel braking. */
        public braking: boolean;

        /**
         * Creates a new RobotWheel instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotWheel instance
         */
        public static create(properties?: proto.IRobotWheel): proto.RobotWheel;

        /**
         * Encodes the specified RobotWheel message. Does not implicitly {@link proto.RobotWheel.verify|verify} messages.
         * @param message RobotWheel message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotWheel, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotWheel message, length delimited. Does not implicitly {@link proto.RobotWheel.verify|verify} messages.
         * @param message RobotWheel message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotWheel, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotWheel message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotWheel
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotWheel;

        /**
         * Decodes a RobotWheel message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotWheel
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotWheel;

        /**
         * Verifies a RobotWheel message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotWheel message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotWheel
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotWheel;

        /**
         * Creates a plain object from a RobotWheel message. Also converts values to other types if specified.
         * @param message RobotWheel
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotWheel, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotWheel to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotWheel
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotWheels. */
    interface IRobotWheels {

        /** RobotWheels rightFront */
        rightFront?: (proto.IRobotWheel|null);

        /** RobotWheels rightBack */
        rightBack?: (proto.IRobotWheel|null);

        /** RobotWheels leftBack */
        leftBack?: (proto.IRobotWheel|null);

        /** RobotWheels leftFront */
        leftFront?: (proto.IRobotWheel|null);
    }

    /** Represents a RobotWheels. */
    class RobotWheels implements IRobotWheels {

        /**
         * Constructs a new RobotWheels.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotWheels);

        /** RobotWheels rightFront. */
        public rightFront?: (proto.IRobotWheel|null);

        /** RobotWheels rightBack. */
        public rightBack?: (proto.IRobotWheel|null);

        /** RobotWheels leftBack. */
        public leftBack?: (proto.IRobotWheel|null);

        /** RobotWheels leftFront. */
        public leftFront?: (proto.IRobotWheel|null);

        /**
         * Creates a new RobotWheels instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotWheels instance
         */
        public static create(properties?: proto.IRobotWheels): proto.RobotWheels;

        /**
         * Encodes the specified RobotWheels message. Does not implicitly {@link proto.RobotWheels.verify|verify} messages.
         * @param message RobotWheels message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotWheels, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotWheels message, length delimited. Does not implicitly {@link proto.RobotWheels.verify|verify} messages.
         * @param message RobotWheels message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotWheels, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotWheels message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotWheels
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotWheels;

        /**
         * Decodes a RobotWheels message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotWheels
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotWheels;

        /**
         * Verifies a RobotWheels message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotWheels message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotWheels
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotWheels;

        /**
         * Creates a plain object from a RobotWheels message. Also converts values to other types if specified.
         * @param message RobotWheels
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotWheels, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotWheels to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotWheels
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotProcessedFeedback. */
    interface IRobotProcessedFeedback {

        /** RobotProcessedFeedback ballSensorSeesBall */
        ballSensorSeesBall?: (boolean|null);

        /** RobotProcessedFeedback ballPosition */
        ballPosition?: (number|null);

        /** RobotProcessedFeedback ballSensorIsWorking */
        ballSensorIsWorking?: (boolean|null);

        /** RobotProcessedFeedback dribblerSeesBall */
        dribblerSeesBall?: (boolean|null);

        /** RobotProcessedFeedback xsensIsCalibrated */
        xsensIsCalibrated?: (boolean|null);

        /** RobotProcessedFeedback capacitorIsCharged */
        capacitorIsCharged?: (boolean|null);

        /** RobotProcessedFeedback wheelInformation */
        wheelInformation?: (proto.IRobotWheels|null);

        /** RobotProcessedFeedback batteryLevel */
        batteryLevel?: (number|null);

        /** RobotProcessedFeedback signalStrength */
        signalStrength?: (number|null);
    }

    /** Represents a RobotProcessedFeedback. */
    class RobotProcessedFeedback implements IRobotProcessedFeedback {

        /**
         * Constructs a new RobotProcessedFeedback.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotProcessedFeedback);

        /** RobotProcessedFeedback ballSensorSeesBall. */
        public ballSensorSeesBall: boolean;

        /** RobotProcessedFeedback ballPosition. */
        public ballPosition: number;

        /** RobotProcessedFeedback ballSensorIsWorking. */
        public ballSensorIsWorking: boolean;

        /** RobotProcessedFeedback dribblerSeesBall. */
        public dribblerSeesBall: boolean;

        /** RobotProcessedFeedback xsensIsCalibrated. */
        public xsensIsCalibrated: boolean;

        /** RobotProcessedFeedback capacitorIsCharged. */
        public capacitorIsCharged: boolean;

        /** RobotProcessedFeedback wheelInformation. */
        public wheelInformation?: (proto.IRobotWheels|null);

        /** RobotProcessedFeedback batteryLevel. */
        public batteryLevel: number;

        /** RobotProcessedFeedback signalStrength. */
        public signalStrength: number;

        /**
         * Creates a new RobotProcessedFeedback instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotProcessedFeedback instance
         */
        public static create(properties?: proto.IRobotProcessedFeedback): proto.RobotProcessedFeedback;

        /**
         * Encodes the specified RobotProcessedFeedback message. Does not implicitly {@link proto.RobotProcessedFeedback.verify|verify} messages.
         * @param message RobotProcessedFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotProcessedFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotProcessedFeedback message, length delimited. Does not implicitly {@link proto.RobotProcessedFeedback.verify|verify} messages.
         * @param message RobotProcessedFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotProcessedFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotProcessedFeedback message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotProcessedFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotProcessedFeedback;

        /**
         * Decodes a RobotProcessedFeedback message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotProcessedFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotProcessedFeedback;

        /**
         * Verifies a RobotProcessedFeedback message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotProcessedFeedback message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotProcessedFeedback
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotProcessedFeedback;

        /**
         * Creates a plain object from a RobotProcessedFeedback message. Also converts values to other types if specified.
         * @param message RobotProcessedFeedback
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotProcessedFeedback, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotProcessedFeedback to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotProcessedFeedback
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a FeedbackOnlyRobot. */
    interface IFeedbackOnlyRobot {

        /** FeedbackOnlyRobot id */
        id?: (number|null);

        /** FeedbackOnlyRobot feedback */
        feedback?: (proto.IRobotProcessedFeedback|null);
    }

    /** Represents a FeedbackOnlyRobot. */
    class FeedbackOnlyRobot implements IFeedbackOnlyRobot {

        /**
         * Constructs a new FeedbackOnlyRobot.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IFeedbackOnlyRobot);

        /** FeedbackOnlyRobot id. */
        public id: number;

        /** FeedbackOnlyRobot feedback. */
        public feedback?: (proto.IRobotProcessedFeedback|null);

        /**
         * Creates a new FeedbackOnlyRobot instance using the specified properties.
         * @param [properties] Properties to set
         * @returns FeedbackOnlyRobot instance
         */
        public static create(properties?: proto.IFeedbackOnlyRobot): proto.FeedbackOnlyRobot;

        /**
         * Encodes the specified FeedbackOnlyRobot message. Does not implicitly {@link proto.FeedbackOnlyRobot.verify|verify} messages.
         * @param message FeedbackOnlyRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IFeedbackOnlyRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified FeedbackOnlyRobot message, length delimited. Does not implicitly {@link proto.FeedbackOnlyRobot.verify|verify} messages.
         * @param message FeedbackOnlyRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IFeedbackOnlyRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a FeedbackOnlyRobot message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns FeedbackOnlyRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.FeedbackOnlyRobot;

        /**
         * Decodes a FeedbackOnlyRobot message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns FeedbackOnlyRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.FeedbackOnlyRobot;

        /**
         * Verifies a FeedbackOnlyRobot message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a FeedbackOnlyRobot message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns FeedbackOnlyRobot
         */
        public static fromObject(object: { [k: string]: any }): proto.FeedbackOnlyRobot;

        /**
         * Creates a plain object from a FeedbackOnlyRobot message. Also converts values to other types if specified.
         * @param message FeedbackOnlyRobot
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.FeedbackOnlyRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this FeedbackOnlyRobot to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for FeedbackOnlyRobot
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a State. */
    interface IState {

        /** State lastSeenWorld */
        lastSeenWorld?: (proto.IWorld|null);

        /** State commandExtrapolatedWorld */
        commandExtrapolatedWorld?: (proto.IWorld|null);

        /** State ballCameraWorld */
        ballCameraWorld?: (proto.IWorld|null);

        /** State blueRobotParameters */
        blueRobotParameters?: (proto.ITeamParameters|null);

        /** State yellowRobotParameters */
        yellowRobotParameters?: (proto.ITeamParameters|null);

        /** State field */
        field?: (proto.ISSL_GeometryData|null);

        /** State referee */
        referee?: (proto.ISSL_Referee|null);

        /** State processedVisionPackets */
        processedVisionPackets?: (proto.ISSL_WrapperPacket[]|null);

        /** State processedRefereePackets */
        processedRefereePackets?: (proto.ISSL_Referee[]|null);
    }

    /** Represents a State. */
    class State implements IState {

        /**
         * Constructs a new State.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IState);

        /** State lastSeenWorld. */
        public lastSeenWorld?: (proto.IWorld|null);

        /** State commandExtrapolatedWorld. */
        public commandExtrapolatedWorld?: (proto.IWorld|null);

        /** State ballCameraWorld. */
        public ballCameraWorld?: (proto.IWorld|null);

        /** State blueRobotParameters. */
        public blueRobotParameters?: (proto.ITeamParameters|null);

        /** State yellowRobotParameters. */
        public yellowRobotParameters?: (proto.ITeamParameters|null);

        /** State field. */
        public field?: (proto.ISSL_GeometryData|null);

        /** State referee. */
        public referee?: (proto.ISSL_Referee|null);

        /** State processedVisionPackets. */
        public processedVisionPackets: proto.ISSL_WrapperPacket[];

        /** State processedRefereePackets. */
        public processedRefereePackets: proto.ISSL_Referee[];

        /**
         * Creates a new State instance using the specified properties.
         * @param [properties] Properties to set
         * @returns State instance
         */
        public static create(properties?: proto.IState): proto.State;

        /**
         * Encodes the specified State message. Does not implicitly {@link proto.State.verify|verify} messages.
         * @param message State message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified State message, length delimited. Does not implicitly {@link proto.State.verify|verify} messages.
         * @param message State message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a State message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns State
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.State;

        /**
         * Decodes a State message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns State
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.State;

        /**
         * Verifies a State message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a State message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns State
         */
        public static fromObject(object: { [k: string]: any }): proto.State;

        /**
         * Creates a plain object from a State message. Also converts values to other types if specified.
         * @param message State
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.State, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this State to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for State
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a ModuleState. */
    interface IModuleState {

        /** ModuleState systemState */
        systemState?: (proto.IState|null);

        /** ModuleState handshakes */
        handshakes?: (proto.IHandshake[]|null);
    }

    /** Represents a ModuleState. */
    class ModuleState implements IModuleState {

        /**
         * Constructs a new ModuleState.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IModuleState);

        /** ModuleState systemState. */
        public systemState?: (proto.IState|null);

        /** ModuleState handshakes. */
        public handshakes: proto.IHandshake[];

        /**
         * Creates a new ModuleState instance using the specified properties.
         * @param [properties] Properties to set
         * @returns ModuleState instance
         */
        public static create(properties?: proto.IModuleState): proto.ModuleState;

        /**
         * Encodes the specified ModuleState message. Does not implicitly {@link proto.ModuleState.verify|verify} messages.
         * @param message ModuleState message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IModuleState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified ModuleState message, length delimited. Does not implicitly {@link proto.ModuleState.verify|verify} messages.
         * @param message ModuleState message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IModuleState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a ModuleState message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns ModuleState
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.ModuleState;

        /**
         * Decodes a ModuleState message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns ModuleState
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.ModuleState;

        /**
         * Verifies a ModuleState message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a ModuleState message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns ModuleState
         */
        public static fromObject(object: { [k: string]: any }): proto.ModuleState;

        /**
         * Creates a plain object from a ModuleState message. Also converts values to other types if specified.
         * @param message ModuleState
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.ModuleState, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this ModuleState to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for ModuleState
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotParameters. */
    interface IRobotParameters {

        /** RobotParameters radius */
        radius?: (number|null);

        /** RobotParameters height */
        height?: (number|null);

        /** RobotParameters frontWidth */
        frontWidth?: (number|null);

        /** RobotParameters dribblerWidth */
        dribblerWidth?: (number|null);

        /** RobotParameters angleOffset */
        angleOffset?: (number|null);
    }

    /** Represents a RobotParameters. */
    class RobotParameters implements IRobotParameters {

        /**
         * Constructs a new RobotParameters.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotParameters);

        /** RobotParameters radius. */
        public radius: number;

        /** RobotParameters height. */
        public height: number;

        /** RobotParameters frontWidth. */
        public frontWidth: number;

        /** RobotParameters dribblerWidth. */
        public dribblerWidth: number;

        /** RobotParameters angleOffset. */
        public angleOffset: number;

        /**
         * Creates a new RobotParameters instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotParameters instance
         */
        public static create(properties?: proto.IRobotParameters): proto.RobotParameters;

        /**
         * Encodes the specified RobotParameters message. Does not implicitly {@link proto.RobotParameters.verify|verify} messages.
         * @param message RobotParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotParameters message, length delimited. Does not implicitly {@link proto.RobotParameters.verify|verify} messages.
         * @param message RobotParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotParameters message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotParameters;

        /**
         * Decodes a RobotParameters message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotParameters;

        /**
         * Verifies a RobotParameters message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotParameters message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotParameters
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotParameters;

        /**
         * Creates a plain object from a RobotParameters message. Also converts values to other types if specified.
         * @param message RobotParameters
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotParameters, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotParameters to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotParameters
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a TeamParameters. */
    interface ITeamParameters {

        /** TeamParameters didChange */
        didChange?: (boolean|null);

        /** TeamParameters parameters */
        parameters?: (proto.IRobotParameters|null);
    }

    /** Represents a TeamParameters. */
    class TeamParameters implements ITeamParameters {

        /**
         * Constructs a new TeamParameters.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ITeamParameters);

        /** TeamParameters didChange. */
        public didChange: boolean;

        /** TeamParameters parameters. */
        public parameters?: (proto.IRobotParameters|null);

        /**
         * Creates a new TeamParameters instance using the specified properties.
         * @param [properties] Properties to set
         * @returns TeamParameters instance
         */
        public static create(properties?: proto.ITeamParameters): proto.TeamParameters;

        /**
         * Encodes the specified TeamParameters message. Does not implicitly {@link proto.TeamParameters.verify|verify} messages.
         * @param message TeamParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ITeamParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified TeamParameters message, length delimited. Does not implicitly {@link proto.TeamParameters.verify|verify} messages.
         * @param message TeamParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ITeamParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a TeamParameters message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns TeamParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.TeamParameters;

        /**
         * Decodes a TeamParameters message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns TeamParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.TeamParameters;

        /**
         * Verifies a TeamParameters message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a TeamParameters message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns TeamParameters
         */
        public static fromObject(object: { [k: string]: any }): proto.TeamParameters;

        /**
         * Creates a plain object from a TeamParameters message. Also converts values to other types if specified.
         * @param message TeamParameters
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.TeamParameters, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this TeamParameters to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for TeamParameters
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_WrapperPacket. */
    interface ISSL_WrapperPacket {

        /** SSL_WrapperPacket detection */
        detection?: (proto.ISSL_DetectionFrame|null);

        /** SSL_WrapperPacket geometry */
        geometry?: (proto.ISSL_GeometryData|null);
    }

    /** Represents a SSL_WrapperPacket. */
    class SSL_WrapperPacket implements ISSL_WrapperPacket {

        /**
         * Constructs a new SSL_WrapperPacket.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_WrapperPacket);

        /** SSL_WrapperPacket detection. */
        public detection?: (proto.ISSL_DetectionFrame|null);

        /** SSL_WrapperPacket geometry. */
        public geometry?: (proto.ISSL_GeometryData|null);

        /**
         * Creates a new SSL_WrapperPacket instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_WrapperPacket instance
         */
        public static create(properties?: proto.ISSL_WrapperPacket): proto.SSL_WrapperPacket;

        /**
         * Encodes the specified SSL_WrapperPacket message. Does not implicitly {@link proto.SSL_WrapperPacket.verify|verify} messages.
         * @param message SSL_WrapperPacket message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_WrapperPacket message, length delimited. Does not implicitly {@link proto.SSL_WrapperPacket.verify|verify} messages.
         * @param message SSL_WrapperPacket message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_WrapperPacket message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_WrapperPacket
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_WrapperPacket;

        /**
         * Decodes a SSL_WrapperPacket message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_WrapperPacket
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_WrapperPacket;

        /**
         * Verifies a SSL_WrapperPacket message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_WrapperPacket message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_WrapperPacket
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_WrapperPacket;

        /**
         * Creates a plain object from a SSL_WrapperPacket message. Also converts values to other types if specified.
         * @param message SSL_WrapperPacket
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_WrapperPacket, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_WrapperPacket to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_WrapperPacket
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_DetectionBall. */
    interface ISSL_DetectionBall {

        /** SSL_DetectionBall confidence */
        confidence: number;

        /** SSL_DetectionBall area */
        area?: (number|null);

        /** SSL_DetectionBall x */
        x: number;

        /** SSL_DetectionBall y */
        y: number;

        /** SSL_DetectionBall z */
        z?: (number|null);

        /** SSL_DetectionBall pixelX */
        pixelX: number;

        /** SSL_DetectionBall pixelY */
        pixelY: number;
    }

    /** Represents a SSL_DetectionBall. */
    class SSL_DetectionBall implements ISSL_DetectionBall {

        /**
         * Constructs a new SSL_DetectionBall.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_DetectionBall);

        /** SSL_DetectionBall confidence. */
        public confidence: number;

        /** SSL_DetectionBall area. */
        public area: number;

        /** SSL_DetectionBall x. */
        public x: number;

        /** SSL_DetectionBall y. */
        public y: number;

        /** SSL_DetectionBall z. */
        public z: number;

        /** SSL_DetectionBall pixelX. */
        public pixelX: number;

        /** SSL_DetectionBall pixelY. */
        public pixelY: number;

        /**
         * Creates a new SSL_DetectionBall instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_DetectionBall instance
         */
        public static create(properties?: proto.ISSL_DetectionBall): proto.SSL_DetectionBall;

        /**
         * Encodes the specified SSL_DetectionBall message. Does not implicitly {@link proto.SSL_DetectionBall.verify|verify} messages.
         * @param message SSL_DetectionBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_DetectionBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_DetectionBall message, length delimited. Does not implicitly {@link proto.SSL_DetectionBall.verify|verify} messages.
         * @param message SSL_DetectionBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_DetectionBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_DetectionBall message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_DetectionBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_DetectionBall;

        /**
         * Decodes a SSL_DetectionBall message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_DetectionBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_DetectionBall;

        /**
         * Verifies a SSL_DetectionBall message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_DetectionBall message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_DetectionBall
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_DetectionBall;

        /**
         * Creates a plain object from a SSL_DetectionBall message. Also converts values to other types if specified.
         * @param message SSL_DetectionBall
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_DetectionBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_DetectionBall to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_DetectionBall
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_DetectionRobot. */
    interface ISSL_DetectionRobot {

        /** SSL_DetectionRobot confidence */
        confidence: number;

        /** SSL_DetectionRobot robotId */
        robotId?: (number|null);

        /** SSL_DetectionRobot x */
        x: number;

        /** SSL_DetectionRobot y */
        y: number;

        /** SSL_DetectionRobot orientation */
        orientation?: (number|null);

        /** SSL_DetectionRobot pixelX */
        pixelX: number;

        /** SSL_DetectionRobot pixelY */
        pixelY: number;

        /** SSL_DetectionRobot height */
        height?: (number|null);
    }

    /** Represents a SSL_DetectionRobot. */
    class SSL_DetectionRobot implements ISSL_DetectionRobot {

        /**
         * Constructs a new SSL_DetectionRobot.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_DetectionRobot);

        /** SSL_DetectionRobot confidence. */
        public confidence: number;

        /** SSL_DetectionRobot robotId. */
        public robotId: number;

        /** SSL_DetectionRobot x. */
        public x: number;

        /** SSL_DetectionRobot y. */
        public y: number;

        /** SSL_DetectionRobot orientation. */
        public orientation: number;

        /** SSL_DetectionRobot pixelX. */
        public pixelX: number;

        /** SSL_DetectionRobot pixelY. */
        public pixelY: number;

        /** SSL_DetectionRobot height. */
        public height: number;

        /**
         * Creates a new SSL_DetectionRobot instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_DetectionRobot instance
         */
        public static create(properties?: proto.ISSL_DetectionRobot): proto.SSL_DetectionRobot;

        /**
         * Encodes the specified SSL_DetectionRobot message. Does not implicitly {@link proto.SSL_DetectionRobot.verify|verify} messages.
         * @param message SSL_DetectionRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_DetectionRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_DetectionRobot message, length delimited. Does not implicitly {@link proto.SSL_DetectionRobot.verify|verify} messages.
         * @param message SSL_DetectionRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_DetectionRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_DetectionRobot message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_DetectionRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_DetectionRobot;

        /**
         * Decodes a SSL_DetectionRobot message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_DetectionRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_DetectionRobot;

        /**
         * Verifies a SSL_DetectionRobot message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_DetectionRobot message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_DetectionRobot
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_DetectionRobot;

        /**
         * Creates a plain object from a SSL_DetectionRobot message. Also converts values to other types if specified.
         * @param message SSL_DetectionRobot
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_DetectionRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_DetectionRobot to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_DetectionRobot
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_DetectionFrame. */
    interface ISSL_DetectionFrame {

        /** SSL_DetectionFrame frameNumber */
        frameNumber: number;

        /** SSL_DetectionFrame tCapture */
        tCapture: number;

        /** SSL_DetectionFrame tSent */
        tSent: number;

        /** SSL_DetectionFrame cameraId */
        cameraId: number;

        /** SSL_DetectionFrame balls */
        balls?: (proto.ISSL_DetectionBall[]|null);

        /** SSL_DetectionFrame robotsYellow */
        robotsYellow?: (proto.ISSL_DetectionRobot[]|null);

        /** SSL_DetectionFrame robotsBlue */
        robotsBlue?: (proto.ISSL_DetectionRobot[]|null);
    }

    /** Represents a SSL_DetectionFrame. */
    class SSL_DetectionFrame implements ISSL_DetectionFrame {

        /**
         * Constructs a new SSL_DetectionFrame.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_DetectionFrame);

        /** SSL_DetectionFrame frameNumber. */
        public frameNumber: number;

        /** SSL_DetectionFrame tCapture. */
        public tCapture: number;

        /** SSL_DetectionFrame tSent. */
        public tSent: number;

        /** SSL_DetectionFrame cameraId. */
        public cameraId: number;

        /** SSL_DetectionFrame balls. */
        public balls: proto.ISSL_DetectionBall[];

        /** SSL_DetectionFrame robotsYellow. */
        public robotsYellow: proto.ISSL_DetectionRobot[];

        /** SSL_DetectionFrame robotsBlue. */
        public robotsBlue: proto.ISSL_DetectionRobot[];

        /**
         * Creates a new SSL_DetectionFrame instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_DetectionFrame instance
         */
        public static create(properties?: proto.ISSL_DetectionFrame): proto.SSL_DetectionFrame;

        /**
         * Encodes the specified SSL_DetectionFrame message. Does not implicitly {@link proto.SSL_DetectionFrame.verify|verify} messages.
         * @param message SSL_DetectionFrame message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_DetectionFrame, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_DetectionFrame message, length delimited. Does not implicitly {@link proto.SSL_DetectionFrame.verify|verify} messages.
         * @param message SSL_DetectionFrame message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_DetectionFrame, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_DetectionFrame message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_DetectionFrame
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_DetectionFrame;

        /**
         * Decodes a SSL_DetectionFrame message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_DetectionFrame
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_DetectionFrame;

        /**
         * Verifies a SSL_DetectionFrame message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_DetectionFrame message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_DetectionFrame
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_DetectionFrame;

        /**
         * Creates a plain object from a SSL_DetectionFrame message. Also converts values to other types if specified.
         * @param message SSL_DetectionFrame
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_DetectionFrame, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_DetectionFrame to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_DetectionFrame
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_FieldLineSegment. */
    interface ISSL_FieldLineSegment {

        /** SSL_FieldLineSegment name */
        name: string;

        /** SSL_FieldLineSegment p1 */
        p1: proto.IVector2f;

        /** SSL_FieldLineSegment p2 */
        p2: proto.IVector2f;

        /** SSL_FieldLineSegment thickness */
        thickness: number;

        /** SSL_FieldLineSegment type */
        type?: (proto.SSL_FieldShapeType|null);
    }

    /** Represents a SSL_FieldLineSegment. */
    class SSL_FieldLineSegment implements ISSL_FieldLineSegment {

        /**
         * Constructs a new SSL_FieldLineSegment.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_FieldLineSegment);

        /** SSL_FieldLineSegment name. */
        public name: string;

        /** SSL_FieldLineSegment p1. */
        public p1: proto.IVector2f;

        /** SSL_FieldLineSegment p2. */
        public p2: proto.IVector2f;

        /** SSL_FieldLineSegment thickness. */
        public thickness: number;

        /** SSL_FieldLineSegment type. */
        public type: proto.SSL_FieldShapeType;

        /**
         * Creates a new SSL_FieldLineSegment instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_FieldLineSegment instance
         */
        public static create(properties?: proto.ISSL_FieldLineSegment): proto.SSL_FieldLineSegment;

        /**
         * Encodes the specified SSL_FieldLineSegment message. Does not implicitly {@link proto.SSL_FieldLineSegment.verify|verify} messages.
         * @param message SSL_FieldLineSegment message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_FieldLineSegment, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_FieldLineSegment message, length delimited. Does not implicitly {@link proto.SSL_FieldLineSegment.verify|verify} messages.
         * @param message SSL_FieldLineSegment message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_FieldLineSegment, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_FieldLineSegment message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_FieldLineSegment
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_FieldLineSegment;

        /**
         * Decodes a SSL_FieldLineSegment message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_FieldLineSegment
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_FieldLineSegment;

        /**
         * Verifies a SSL_FieldLineSegment message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_FieldLineSegment message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_FieldLineSegment
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_FieldLineSegment;

        /**
         * Creates a plain object from a SSL_FieldLineSegment message. Also converts values to other types if specified.
         * @param message SSL_FieldLineSegment
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_FieldLineSegment, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_FieldLineSegment to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_FieldLineSegment
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_FieldCircularArc. */
    interface ISSL_FieldCircularArc {

        /** SSL_FieldCircularArc name */
        name: string;

        /** SSL_FieldCircularArc center */
        center: proto.IVector2f;

        /** SSL_FieldCircularArc radius */
        radius: number;

        /** SSL_FieldCircularArc a1 */
        a1: number;

        /** SSL_FieldCircularArc a2 */
        a2: number;

        /** SSL_FieldCircularArc thickness */
        thickness: number;

        /** SSL_FieldCircularArc type */
        type?: (proto.SSL_FieldShapeType|null);
    }

    /** Represents a SSL_FieldCircularArc. */
    class SSL_FieldCircularArc implements ISSL_FieldCircularArc {

        /**
         * Constructs a new SSL_FieldCircularArc.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_FieldCircularArc);

        /** SSL_FieldCircularArc name. */
        public name: string;

        /** SSL_FieldCircularArc center. */
        public center: proto.IVector2f;

        /** SSL_FieldCircularArc radius. */
        public radius: number;

        /** SSL_FieldCircularArc a1. */
        public a1: number;

        /** SSL_FieldCircularArc a2. */
        public a2: number;

        /** SSL_FieldCircularArc thickness. */
        public thickness: number;

        /** SSL_FieldCircularArc type. */
        public type: proto.SSL_FieldShapeType;

        /**
         * Creates a new SSL_FieldCircularArc instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_FieldCircularArc instance
         */
        public static create(properties?: proto.ISSL_FieldCircularArc): proto.SSL_FieldCircularArc;

        /**
         * Encodes the specified SSL_FieldCircularArc message. Does not implicitly {@link proto.SSL_FieldCircularArc.verify|verify} messages.
         * @param message SSL_FieldCircularArc message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_FieldCircularArc, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_FieldCircularArc message, length delimited. Does not implicitly {@link proto.SSL_FieldCircularArc.verify|verify} messages.
         * @param message SSL_FieldCircularArc message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_FieldCircularArc, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_FieldCircularArc message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_FieldCircularArc
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_FieldCircularArc;

        /**
         * Decodes a SSL_FieldCircularArc message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_FieldCircularArc
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_FieldCircularArc;

        /**
         * Verifies a SSL_FieldCircularArc message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_FieldCircularArc message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_FieldCircularArc
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_FieldCircularArc;

        /**
         * Creates a plain object from a SSL_FieldCircularArc message. Also converts values to other types if specified.
         * @param message SSL_FieldCircularArc
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_FieldCircularArc, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_FieldCircularArc to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_FieldCircularArc
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_GeometryFieldSize. */
    interface ISSL_GeometryFieldSize {

        /** SSL_GeometryFieldSize fieldLength */
        fieldLength: number;

        /** SSL_GeometryFieldSize fieldWidth */
        fieldWidth: number;

        /** SSL_GeometryFieldSize goalWidth */
        goalWidth: number;

        /** SSL_GeometryFieldSize goalDepth */
        goalDepth: number;

        /** SSL_GeometryFieldSize boundaryWidth */
        boundaryWidth: number;

        /** SSL_GeometryFieldSize fieldLines */
        fieldLines?: (proto.ISSL_FieldLineSegment[]|null);

        /** SSL_GeometryFieldSize fieldArcs */
        fieldArcs?: (proto.ISSL_FieldCircularArc[]|null);

        /** SSL_GeometryFieldSize penaltyAreaDepth */
        penaltyAreaDepth?: (number|null);

        /** SSL_GeometryFieldSize penaltyAreaWidth */
        penaltyAreaWidth?: (number|null);
    }

    /** Represents a SSL_GeometryFieldSize. */
    class SSL_GeometryFieldSize implements ISSL_GeometryFieldSize {

        /**
         * Constructs a new SSL_GeometryFieldSize.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_GeometryFieldSize);

        /** SSL_GeometryFieldSize fieldLength. */
        public fieldLength: number;

        /** SSL_GeometryFieldSize fieldWidth. */
        public fieldWidth: number;

        /** SSL_GeometryFieldSize goalWidth. */
        public goalWidth: number;

        /** SSL_GeometryFieldSize goalDepth. */
        public goalDepth: number;

        /** SSL_GeometryFieldSize boundaryWidth. */
        public boundaryWidth: number;

        /** SSL_GeometryFieldSize fieldLines. */
        public fieldLines: proto.ISSL_FieldLineSegment[];

        /** SSL_GeometryFieldSize fieldArcs. */
        public fieldArcs: proto.ISSL_FieldCircularArc[];

        /** SSL_GeometryFieldSize penaltyAreaDepth. */
        public penaltyAreaDepth: number;

        /** SSL_GeometryFieldSize penaltyAreaWidth. */
        public penaltyAreaWidth: number;

        /**
         * Creates a new SSL_GeometryFieldSize instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_GeometryFieldSize instance
         */
        public static create(properties?: proto.ISSL_GeometryFieldSize): proto.SSL_GeometryFieldSize;

        /**
         * Encodes the specified SSL_GeometryFieldSize message. Does not implicitly {@link proto.SSL_GeometryFieldSize.verify|verify} messages.
         * @param message SSL_GeometryFieldSize message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_GeometryFieldSize message, length delimited. Does not implicitly {@link proto.SSL_GeometryFieldSize.verify|verify} messages.
         * @param message SSL_GeometryFieldSize message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_GeometryFieldSize
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_GeometryFieldSize;

        /**
         * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_GeometryFieldSize
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_GeometryFieldSize;

        /**
         * Verifies a SSL_GeometryFieldSize message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_GeometryFieldSize message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_GeometryFieldSize
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_GeometryFieldSize;

        /**
         * Creates a plain object from a SSL_GeometryFieldSize message. Also converts values to other types if specified.
         * @param message SSL_GeometryFieldSize
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_GeometryFieldSize, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_GeometryFieldSize to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_GeometryFieldSize
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_GeometryCameraCalibration. */
    interface ISSL_GeometryCameraCalibration {

        /** SSL_GeometryCameraCalibration cameraId */
        cameraId: number;

        /** SSL_GeometryCameraCalibration focalLength */
        focalLength: number;

        /** SSL_GeometryCameraCalibration principalPointX */
        principalPointX: number;

        /** SSL_GeometryCameraCalibration principalPointY */
        principalPointY: number;

        /** SSL_GeometryCameraCalibration distortion */
        distortion: number;

        /** SSL_GeometryCameraCalibration q0 */
        q0: number;

        /** SSL_GeometryCameraCalibration q1 */
        q1: number;

        /** SSL_GeometryCameraCalibration q2 */
        q2: number;

        /** SSL_GeometryCameraCalibration q3 */
        q3: number;

        /** SSL_GeometryCameraCalibration tx */
        tx: number;

        /** SSL_GeometryCameraCalibration ty */
        ty: number;

        /** SSL_GeometryCameraCalibration tz */
        tz: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTx */
        derivedCameraWorldTx?: (number|null);

        /** SSL_GeometryCameraCalibration derivedCameraWorldTy */
        derivedCameraWorldTy?: (number|null);

        /** SSL_GeometryCameraCalibration derivedCameraWorldTz */
        derivedCameraWorldTz?: (number|null);

        /** SSL_GeometryCameraCalibration pixelImageWidth */
        pixelImageWidth?: (number|null);

        /** SSL_GeometryCameraCalibration pixelImageHeight */
        pixelImageHeight?: (number|null);
    }

    /** Represents a SSL_GeometryCameraCalibration. */
    class SSL_GeometryCameraCalibration implements ISSL_GeometryCameraCalibration {

        /**
         * Constructs a new SSL_GeometryCameraCalibration.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_GeometryCameraCalibration);

        /** SSL_GeometryCameraCalibration cameraId. */
        public cameraId: number;

        /** SSL_GeometryCameraCalibration focalLength. */
        public focalLength: number;

        /** SSL_GeometryCameraCalibration principalPointX. */
        public principalPointX: number;

        /** SSL_GeometryCameraCalibration principalPointY. */
        public principalPointY: number;

        /** SSL_GeometryCameraCalibration distortion. */
        public distortion: number;

        /** SSL_GeometryCameraCalibration q0. */
        public q0: number;

        /** SSL_GeometryCameraCalibration q1. */
        public q1: number;

        /** SSL_GeometryCameraCalibration q2. */
        public q2: number;

        /** SSL_GeometryCameraCalibration q3. */
        public q3: number;

        /** SSL_GeometryCameraCalibration tx. */
        public tx: number;

        /** SSL_GeometryCameraCalibration ty. */
        public ty: number;

        /** SSL_GeometryCameraCalibration tz. */
        public tz: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTx. */
        public derivedCameraWorldTx: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTy. */
        public derivedCameraWorldTy: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTz. */
        public derivedCameraWorldTz: number;

        /** SSL_GeometryCameraCalibration pixelImageWidth. */
        public pixelImageWidth: number;

        /** SSL_GeometryCameraCalibration pixelImageHeight. */
        public pixelImageHeight: number;

        /**
         * Creates a new SSL_GeometryCameraCalibration instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_GeometryCameraCalibration instance
         */
        public static create(properties?: proto.ISSL_GeometryCameraCalibration): proto.SSL_GeometryCameraCalibration;

        /**
         * Encodes the specified SSL_GeometryCameraCalibration message. Does not implicitly {@link proto.SSL_GeometryCameraCalibration.verify|verify} messages.
         * @param message SSL_GeometryCameraCalibration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_GeometryCameraCalibration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_GeometryCameraCalibration message, length delimited. Does not implicitly {@link proto.SSL_GeometryCameraCalibration.verify|verify} messages.
         * @param message SSL_GeometryCameraCalibration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_GeometryCameraCalibration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_GeometryCameraCalibration message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_GeometryCameraCalibration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_GeometryCameraCalibration;

        /**
         * Decodes a SSL_GeometryCameraCalibration message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_GeometryCameraCalibration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_GeometryCameraCalibration;

        /**
         * Verifies a SSL_GeometryCameraCalibration message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_GeometryCameraCalibration message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_GeometryCameraCalibration
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_GeometryCameraCalibration;

        /**
         * Creates a plain object from a SSL_GeometryCameraCalibration message. Also converts values to other types if specified.
         * @param message SSL_GeometryCameraCalibration
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_GeometryCameraCalibration, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_GeometryCameraCalibration to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_GeometryCameraCalibration
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_GeometryData. */
    interface ISSL_GeometryData {

        /** SSL_GeometryData field */
        field: proto.ISSL_GeometryFieldSize;

        /** SSL_GeometryData calib */
        calib?: (proto.ISSL_GeometryCameraCalibration[]|null);
    }

    /** Represents a SSL_GeometryData. */
    class SSL_GeometryData implements ISSL_GeometryData {

        /**
         * Constructs a new SSL_GeometryData.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_GeometryData);

        /** SSL_GeometryData field. */
        public field: proto.ISSL_GeometryFieldSize;

        /** SSL_GeometryData calib. */
        public calib: proto.ISSL_GeometryCameraCalibration[];

        /**
         * Creates a new SSL_GeometryData instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_GeometryData instance
         */
        public static create(properties?: proto.ISSL_GeometryData): proto.SSL_GeometryData;

        /**
         * Encodes the specified SSL_GeometryData message. Does not implicitly {@link proto.SSL_GeometryData.verify|verify} messages.
         * @param message SSL_GeometryData message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_GeometryData message, length delimited. Does not implicitly {@link proto.SSL_GeometryData.verify|verify} messages.
         * @param message SSL_GeometryData message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_GeometryData message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_GeometryData
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_GeometryData;

        /**
         * Decodes a SSL_GeometryData message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_GeometryData
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_GeometryData;

        /**
         * Verifies a SSL_GeometryData message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_GeometryData message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_GeometryData
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_GeometryData;

        /**
         * Creates a plain object from a SSL_GeometryData message. Also converts values to other types if specified.
         * @param message SSL_GeometryData
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_GeometryData, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_GeometryData to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_GeometryData
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** SSL_FieldShapeType enum. */
    enum SSL_FieldShapeType {
        Undefined = 0,
        CenterCircle = 1,
        TopTouchLine = 2,
        BottomTouchLine = 3,
        LeftGoalLine = 4,
        RightGoalLine = 5,
        HalfwayLine = 6,
        CenterLine = 7,
        LeftPenaltyStretch = 8,
        RightPenaltyStretch = 9,
        LeftFieldLeftPenaltyStretch = 10,
        LeftFieldRightPenaltyStretch = 11,
        RightFieldLeftPenaltyStretch = 12,
        RightFieldRightPenaltyStretch = 13
    }

    /** Properties of a SSL_Referee. */
    interface ISSL_Referee {

        /** SSL_Referee packetTimestamp */
        packetTimestamp: (number|Long);

        /** SSL_Referee stage */
        stage: proto.SSL_Referee.Stage;

        /** SSL_Referee stageTimeLeft */
        stageTimeLeft?: (number|null);

        /** SSL_Referee command */
        command: proto.SSL_Referee.Command;

        /** SSL_Referee commandCounter */
        commandCounter: number;

        /** SSL_Referee commandTimestamp */
        commandTimestamp: (number|Long);

        /** SSL_Referee yellow */
        yellow: proto.SSL_Referee.ITeamInfo;

        /** SSL_Referee blue */
        blue: proto.SSL_Referee.ITeamInfo;

        /** SSL_Referee designatedPosition */
        designatedPosition?: (proto.SSL_Referee.IPoint|null);

        /** SSL_Referee blueTeamOnPositiveHalf */
        blueTeamOnPositiveHalf?: (boolean|null);

        /** SSL_Referee nextCommand */
        nextCommand?: (proto.SSL_Referee.Command|null);

        /** SSL_Referee gameEvents */
        gameEvents?: (proto.IGameEvent[]|null);

        /** SSL_Referee gameEventProposals */
        gameEventProposals?: (proto.IGameEventProposalGroup[]|null);

        /** SSL_Referee currentActionTimeRemaining */
        currentActionTimeRemaining?: (number|null);
    }

    /** Represents a SSL_Referee. */
    class SSL_Referee implements ISSL_Referee {

        /**
         * Constructs a new SSL_Referee.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_Referee);

        /** SSL_Referee packetTimestamp. */
        public packetTimestamp: (number|Long);

        /** SSL_Referee stage. */
        public stage: proto.SSL_Referee.Stage;

        /** SSL_Referee stageTimeLeft. */
        public stageTimeLeft: number;

        /** SSL_Referee command. */
        public command: proto.SSL_Referee.Command;

        /** SSL_Referee commandCounter. */
        public commandCounter: number;

        /** SSL_Referee commandTimestamp. */
        public commandTimestamp: (number|Long);

        /** SSL_Referee yellow. */
        public yellow: proto.SSL_Referee.ITeamInfo;

        /** SSL_Referee blue. */
        public blue: proto.SSL_Referee.ITeamInfo;

        /** SSL_Referee designatedPosition. */
        public designatedPosition?: (proto.SSL_Referee.IPoint|null);

        /** SSL_Referee blueTeamOnPositiveHalf. */
        public blueTeamOnPositiveHalf: boolean;

        /** SSL_Referee nextCommand. */
        public nextCommand: proto.SSL_Referee.Command;

        /** SSL_Referee gameEvents. */
        public gameEvents: proto.IGameEvent[];

        /** SSL_Referee gameEventProposals. */
        public gameEventProposals: proto.IGameEventProposalGroup[];

        /** SSL_Referee currentActionTimeRemaining. */
        public currentActionTimeRemaining: number;

        /**
         * Creates a new SSL_Referee instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_Referee instance
         */
        public static create(properties?: proto.ISSL_Referee): proto.SSL_Referee;

        /**
         * Encodes the specified SSL_Referee message. Does not implicitly {@link proto.SSL_Referee.verify|verify} messages.
         * @param message SSL_Referee message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_Referee, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_Referee message, length delimited. Does not implicitly {@link proto.SSL_Referee.verify|verify} messages.
         * @param message SSL_Referee message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_Referee, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_Referee message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_Referee
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_Referee;

        /**
         * Decodes a SSL_Referee message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_Referee
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_Referee;

        /**
         * Verifies a SSL_Referee message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_Referee message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_Referee
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_Referee;

        /**
         * Creates a plain object from a SSL_Referee message. Also converts values to other types if specified.
         * @param message SSL_Referee
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_Referee, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_Referee to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_Referee
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace SSL_Referee {

        /** Stage enum. */
        enum Stage {
            NORMAL_FIRST_HALF_PRE = 0,
            NORMAL_FIRST_HALF = 1,
            NORMAL_HALF_TIME = 2,
            NORMAL_SECOND_HALF_PRE = 3,
            NORMAL_SECOND_HALF = 4,
            EXTRA_TIME_BREAK = 5,
            EXTRA_FIRST_HALF_PRE = 6,
            EXTRA_FIRST_HALF = 7,
            EXTRA_HALF_TIME = 8,
            EXTRA_SECOND_HALF_PRE = 9,
            EXTRA_SECOND_HALF = 10,
            PENALTY_SHOOTOUT_BREAK = 11,
            PENALTY_SHOOTOUT = 12,
            POST_GAME = 13
        }

        /** Command enum. */
        enum Command {
            HALT = 0,
            STOP = 1,
            NORMAL_START = 2,
            FORCE_START = 3,
            PREPARE_KICKOFF_YELLOW = 4,
            PREPARE_KICKOFF_BLUE = 5,
            PREPARE_PENALTY_YELLOW = 6,
            PREPARE_PENALTY_BLUE = 7,
            DIRECT_FREE_YELLOW = 8,
            DIRECT_FREE_BLUE = 9,
            INDIRECT_FREE_YELLOW = 10,
            INDIRECT_FREE_BLUE = 11,
            TIMEOUT_YELLOW = 12,
            TIMEOUT_BLUE = 13,
            BALL_PLACEMENT_YELLOW = 16,
            BALL_PLACEMENT_BLUE = 17
        }

        /** Properties of a TeamInfo. */
        interface ITeamInfo {

            /** TeamInfo name */
            name: string;

            /** TeamInfo score */
            score: number;

            /** TeamInfo redCards */
            redCards: number;

            /** TeamInfo yellowCardTimes */
            yellowCardTimes?: (number[]|null);

            /** TeamInfo yellowCards */
            yellowCards: number;

            /** TeamInfo timeouts */
            timeouts: number;

            /** TeamInfo timeoutTime */
            timeoutTime: number;

            /** TeamInfo goalkeeper */
            goalkeeper: number;

            /** TeamInfo foulCounter */
            foulCounter?: (number|null);

            /** TeamInfo ballPlacementFailures */
            ballPlacementFailures?: (number|null);

            /** TeamInfo canPlaceBall */
            canPlaceBall?: (boolean|null);

            /** TeamInfo maxAllowedBots */
            maxAllowedBots?: (number|null);

            /** TeamInfo botSubstitutionIntent */
            botSubstitutionIntent?: (boolean|null);

            /** TeamInfo ballPlacementFailuresReached */
            ballPlacementFailuresReached?: (boolean|null);
        }

        /** Represents a TeamInfo. */
        class TeamInfo implements ITeamInfo {

            /**
             * Constructs a new TeamInfo.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.SSL_Referee.ITeamInfo);

            /** TeamInfo name. */
            public name: string;

            /** TeamInfo score. */
            public score: number;

            /** TeamInfo redCards. */
            public redCards: number;

            /** TeamInfo yellowCardTimes. */
            public yellowCardTimes: number[];

            /** TeamInfo yellowCards. */
            public yellowCards: number;

            /** TeamInfo timeouts. */
            public timeouts: number;

            /** TeamInfo timeoutTime. */
            public timeoutTime: number;

            /** TeamInfo goalkeeper. */
            public goalkeeper: number;

            /** TeamInfo foulCounter. */
            public foulCounter: number;

            /** TeamInfo ballPlacementFailures. */
            public ballPlacementFailures: number;

            /** TeamInfo canPlaceBall. */
            public canPlaceBall: boolean;

            /** TeamInfo maxAllowedBots. */
            public maxAllowedBots: number;

            /** TeamInfo botSubstitutionIntent. */
            public botSubstitutionIntent: boolean;

            /** TeamInfo ballPlacementFailuresReached. */
            public ballPlacementFailuresReached: boolean;

            /**
             * Creates a new TeamInfo instance using the specified properties.
             * @param [properties] Properties to set
             * @returns TeamInfo instance
             */
            public static create(properties?: proto.SSL_Referee.ITeamInfo): proto.SSL_Referee.TeamInfo;

            /**
             * Encodes the specified TeamInfo message. Does not implicitly {@link proto.SSL_Referee.TeamInfo.verify|verify} messages.
             * @param message TeamInfo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.SSL_Referee.ITeamInfo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified TeamInfo message, length delimited. Does not implicitly {@link proto.SSL_Referee.TeamInfo.verify|verify} messages.
             * @param message TeamInfo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.SSL_Referee.ITeamInfo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a TeamInfo message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns TeamInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_Referee.TeamInfo;

            /**
             * Decodes a TeamInfo message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns TeamInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_Referee.TeamInfo;

            /**
             * Verifies a TeamInfo message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a TeamInfo message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns TeamInfo
             */
            public static fromObject(object: { [k: string]: any }): proto.SSL_Referee.TeamInfo;

            /**
             * Creates a plain object from a TeamInfo message. Also converts values to other types if specified.
             * @param message TeamInfo
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.SSL_Referee.TeamInfo, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this TeamInfo to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for TeamInfo
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Point. */
        interface IPoint {

            /** Point x */
            x: number;

            /** Point y */
            y: number;
        }

        /** Represents a Point. */
        class Point implements IPoint {

            /**
             * Constructs a new Point.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.SSL_Referee.IPoint);

            /** Point x. */
            public x: number;

            /** Point y. */
            public y: number;

            /**
             * Creates a new Point instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Point instance
             */
            public static create(properties?: proto.SSL_Referee.IPoint): proto.SSL_Referee.Point;

            /**
             * Encodes the specified Point message. Does not implicitly {@link proto.SSL_Referee.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.SSL_Referee.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Point message, length delimited. Does not implicitly {@link proto.SSL_Referee.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.SSL_Referee.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Point message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_Referee.Point;

            /**
             * Decodes a Point message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_Referee.Point;

            /**
             * Verifies a Point message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Point message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Point
             */
            public static fromObject(object: { [k: string]: any }): proto.SSL_Referee.Point;

            /**
             * Creates a plain object from a Point message. Also converts values to other types if specified.
             * @param message Point
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.SSL_Referee.Point, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Point to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Point
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a GameEventProposalGroup. */
    interface IGameEventProposalGroup {

        /** GameEventProposalGroup gameEvent */
        gameEvent?: (proto.IGameEvent[]|null);

        /** GameEventProposalGroup accepted */
        accepted?: (boolean|null);
    }

    /** Represents a GameEventProposalGroup. */
    class GameEventProposalGroup implements IGameEventProposalGroup {

        /**
         * Constructs a new GameEventProposalGroup.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IGameEventProposalGroup);

        /** GameEventProposalGroup gameEvent. */
        public gameEvent: proto.IGameEvent[];

        /** GameEventProposalGroup accepted. */
        public accepted: boolean;

        /**
         * Creates a new GameEventProposalGroup instance using the specified properties.
         * @param [properties] Properties to set
         * @returns GameEventProposalGroup instance
         */
        public static create(properties?: proto.IGameEventProposalGroup): proto.GameEventProposalGroup;

        /**
         * Encodes the specified GameEventProposalGroup message. Does not implicitly {@link proto.GameEventProposalGroup.verify|verify} messages.
         * @param message GameEventProposalGroup message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IGameEventProposalGroup, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified GameEventProposalGroup message, length delimited. Does not implicitly {@link proto.GameEventProposalGroup.verify|verify} messages.
         * @param message GameEventProposalGroup message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IGameEventProposalGroup, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a GameEventProposalGroup message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns GameEventProposalGroup
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEventProposalGroup;

        /**
         * Decodes a GameEventProposalGroup message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns GameEventProposalGroup
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEventProposalGroup;

        /**
         * Verifies a GameEventProposalGroup message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a GameEventProposalGroup message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns GameEventProposalGroup
         */
        public static fromObject(object: { [k: string]: any }): proto.GameEventProposalGroup;

        /**
         * Creates a plain object from a GameEventProposalGroup message. Also converts values to other types if specified.
         * @param message GameEventProposalGroup
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.GameEventProposalGroup, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this GameEventProposalGroup to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for GameEventProposalGroup
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a GameEvent. */
    interface IGameEvent {

        /** GameEvent type */
        type?: (proto.GameEvent.Type|null);

        /** GameEvent origin */
        origin?: (string[]|null);

        /** GameEvent ballLeftFieldTouchLine */
        ballLeftFieldTouchLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent ballLeftFieldGoalLine */
        ballLeftFieldGoalLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent aimlessKick */
        aimlessKick?: (proto.GameEvent.IAimlessKick|null);

        /** GameEvent attackerTooCloseToDefenseArea */
        attackerTooCloseToDefenseArea?: (proto.GameEvent.IAttackerTooCloseToDefenseArea|null);

        /** GameEvent defenderInDefenseArea */
        defenderInDefenseArea?: (proto.GameEvent.IDefenderInDefenseArea|null);

        /** GameEvent boundaryCrossing */
        boundaryCrossing?: (proto.GameEvent.IBoundaryCrossing|null);

        /** GameEvent keeperHeldBall */
        keeperHeldBall?: (proto.GameEvent.IKeeperHeldBall|null);

        /** GameEvent botDribbledBallTooFar */
        botDribbledBallTooFar?: (proto.GameEvent.IBotDribbledBallTooFar|null);

        /** GameEvent botPushedBot */
        botPushedBot?: (proto.GameEvent.IBotPushedBot|null);

        /** GameEvent botHeldBallDeliberately */
        botHeldBallDeliberately?: (proto.GameEvent.IBotHeldBallDeliberately|null);

        /** GameEvent botTippedOver */
        botTippedOver?: (proto.GameEvent.IBotTippedOver|null);

        /** GameEvent attackerTouchedBallInDefenseArea */
        attackerTouchedBallInDefenseArea?: (proto.GameEvent.IAttackerTouchedBallInDefenseArea|null);

        /** GameEvent botKickedBallTooFast */
        botKickedBallTooFast?: (proto.GameEvent.IBotKickedBallTooFast|null);

        /** GameEvent botCrashUnique */
        botCrashUnique?: (proto.GameEvent.IBotCrashUnique|null);

        /** GameEvent botCrashDrawn */
        botCrashDrawn?: (proto.GameEvent.IBotCrashDrawn|null);

        /** GameEvent defenderTooCloseToKickPoint */
        defenderTooCloseToKickPoint?: (proto.GameEvent.IDefenderTooCloseToKickPoint|null);

        /** GameEvent botTooFastInStop */
        botTooFastInStop?: (proto.GameEvent.IBotTooFastInStop|null);

        /** GameEvent botInterferedPlacement */
        botInterferedPlacement?: (proto.GameEvent.IBotInterferedPlacement|null);

        /** GameEvent possibleGoal */
        possibleGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent goal */
        goal?: (proto.GameEvent.IGoal|null);

        /** GameEvent invalidGoal */
        invalidGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent attackerDoubleTouchedBall */
        attackerDoubleTouchedBall?: (proto.GameEvent.IAttackerDoubleTouchedBall|null);

        /** GameEvent placementSucceeded */
        placementSucceeded?: (proto.GameEvent.IPlacementSucceeded|null);

        /** GameEvent penaltyKickFailed */
        penaltyKickFailed?: (proto.GameEvent.IPenaltyKickFailed|null);

        /** GameEvent noProgressInGame */
        noProgressInGame?: (proto.GameEvent.INoProgressInGame|null);

        /** GameEvent placementFailed */
        placementFailed?: (proto.GameEvent.IPlacementFailed|null);

        /** GameEvent multipleCards */
        multipleCards?: (proto.GameEvent.IMultipleCards|null);

        /** GameEvent multipleFouls */
        multipleFouls?: (proto.GameEvent.IMultipleFouls|null);

        /** GameEvent botSubstitution */
        botSubstitution?: (proto.GameEvent.IBotSubstitution|null);

        /** GameEvent tooManyRobots */
        tooManyRobots?: (proto.GameEvent.ITooManyRobots|null);

        /** GameEvent challengeFlag */
        challengeFlag?: (proto.GameEvent.IChallengeFlag|null);

        /** GameEvent emergencyStop */
        emergencyStop?: (proto.GameEvent.IEmergencyStop|null);

        /** GameEvent unsportingBehaviorMinor */
        unsportingBehaviorMinor?: (proto.GameEvent.IUnsportingBehaviorMinor|null);

        /** GameEvent unsportingBehaviorMajor */
        unsportingBehaviorMajor?: (proto.GameEvent.IUnsportingBehaviorMajor|null);
    }

    /** Represents a GameEvent. */
    class GameEvent implements IGameEvent {

        /**
         * Constructs a new GameEvent.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IGameEvent);

        /** GameEvent type. */
        public type: proto.GameEvent.Type;

        /** GameEvent origin. */
        public origin: string[];

        /** GameEvent ballLeftFieldTouchLine. */
        public ballLeftFieldTouchLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent ballLeftFieldGoalLine. */
        public ballLeftFieldGoalLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent aimlessKick. */
        public aimlessKick?: (proto.GameEvent.IAimlessKick|null);

        /** GameEvent attackerTooCloseToDefenseArea. */
        public attackerTooCloseToDefenseArea?: (proto.GameEvent.IAttackerTooCloseToDefenseArea|null);

        /** GameEvent defenderInDefenseArea. */
        public defenderInDefenseArea?: (proto.GameEvent.IDefenderInDefenseArea|null);

        /** GameEvent boundaryCrossing. */
        public boundaryCrossing?: (proto.GameEvent.IBoundaryCrossing|null);

        /** GameEvent keeperHeldBall. */
        public keeperHeldBall?: (proto.GameEvent.IKeeperHeldBall|null);

        /** GameEvent botDribbledBallTooFar. */
        public botDribbledBallTooFar?: (proto.GameEvent.IBotDribbledBallTooFar|null);

        /** GameEvent botPushedBot. */
        public botPushedBot?: (proto.GameEvent.IBotPushedBot|null);

        /** GameEvent botHeldBallDeliberately. */
        public botHeldBallDeliberately?: (proto.GameEvent.IBotHeldBallDeliberately|null);

        /** GameEvent botTippedOver. */
        public botTippedOver?: (proto.GameEvent.IBotTippedOver|null);

        /** GameEvent attackerTouchedBallInDefenseArea. */
        public attackerTouchedBallInDefenseArea?: (proto.GameEvent.IAttackerTouchedBallInDefenseArea|null);

        /** GameEvent botKickedBallTooFast. */
        public botKickedBallTooFast?: (proto.GameEvent.IBotKickedBallTooFast|null);

        /** GameEvent botCrashUnique. */
        public botCrashUnique?: (proto.GameEvent.IBotCrashUnique|null);

        /** GameEvent botCrashDrawn. */
        public botCrashDrawn?: (proto.GameEvent.IBotCrashDrawn|null);

        /** GameEvent defenderTooCloseToKickPoint. */
        public defenderTooCloseToKickPoint?: (proto.GameEvent.IDefenderTooCloseToKickPoint|null);

        /** GameEvent botTooFastInStop. */
        public botTooFastInStop?: (proto.GameEvent.IBotTooFastInStop|null);

        /** GameEvent botInterferedPlacement. */
        public botInterferedPlacement?: (proto.GameEvent.IBotInterferedPlacement|null);

        /** GameEvent possibleGoal. */
        public possibleGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent goal. */
        public goal?: (proto.GameEvent.IGoal|null);

        /** GameEvent invalidGoal. */
        public invalidGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent attackerDoubleTouchedBall. */
        public attackerDoubleTouchedBall?: (proto.GameEvent.IAttackerDoubleTouchedBall|null);

        /** GameEvent placementSucceeded. */
        public placementSucceeded?: (proto.GameEvent.IPlacementSucceeded|null);

        /** GameEvent penaltyKickFailed. */
        public penaltyKickFailed?: (proto.GameEvent.IPenaltyKickFailed|null);

        /** GameEvent noProgressInGame. */
        public noProgressInGame?: (proto.GameEvent.INoProgressInGame|null);

        /** GameEvent placementFailed. */
        public placementFailed?: (proto.GameEvent.IPlacementFailed|null);

        /** GameEvent multipleCards. */
        public multipleCards?: (proto.GameEvent.IMultipleCards|null);

        /** GameEvent multipleFouls. */
        public multipleFouls?: (proto.GameEvent.IMultipleFouls|null);

        /** GameEvent botSubstitution. */
        public botSubstitution?: (proto.GameEvent.IBotSubstitution|null);

        /** GameEvent tooManyRobots. */
        public tooManyRobots?: (proto.GameEvent.ITooManyRobots|null);

        /** GameEvent challengeFlag. */
        public challengeFlag?: (proto.GameEvent.IChallengeFlag|null);

        /** GameEvent emergencyStop. */
        public emergencyStop?: (proto.GameEvent.IEmergencyStop|null);

        /** GameEvent unsportingBehaviorMinor. */
        public unsportingBehaviorMinor?: (proto.GameEvent.IUnsportingBehaviorMinor|null);

        /** GameEvent unsportingBehaviorMajor. */
        public unsportingBehaviorMajor?: (proto.GameEvent.IUnsportingBehaviorMajor|null);

        /** GameEvent event. */
        public event?: ("ballLeftFieldTouchLine"|"ballLeftFieldGoalLine"|"aimlessKick"|"attackerTooCloseToDefenseArea"|"defenderInDefenseArea"|"boundaryCrossing"|"keeperHeldBall"|"botDribbledBallTooFar"|"botPushedBot"|"botHeldBallDeliberately"|"botTippedOver"|"attackerTouchedBallInDefenseArea"|"botKickedBallTooFast"|"botCrashUnique"|"botCrashDrawn"|"defenderTooCloseToKickPoint"|"botTooFastInStop"|"botInterferedPlacement"|"possibleGoal"|"goal"|"invalidGoal"|"attackerDoubleTouchedBall"|"placementSucceeded"|"penaltyKickFailed"|"noProgressInGame"|"placementFailed"|"multipleCards"|"multipleFouls"|"botSubstitution"|"tooManyRobots"|"challengeFlag"|"emergencyStop"|"unsportingBehaviorMinor"|"unsportingBehaviorMajor");

        /**
         * Creates a new GameEvent instance using the specified properties.
         * @param [properties] Properties to set
         * @returns GameEvent instance
         */
        public static create(properties?: proto.IGameEvent): proto.GameEvent;

        /**
         * Encodes the specified GameEvent message. Does not implicitly {@link proto.GameEvent.verify|verify} messages.
         * @param message GameEvent message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IGameEvent, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified GameEvent message, length delimited. Does not implicitly {@link proto.GameEvent.verify|verify} messages.
         * @param message GameEvent message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IGameEvent, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a GameEvent message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns GameEvent
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent;

        /**
         * Decodes a GameEvent message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns GameEvent
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent;

        /**
         * Verifies a GameEvent message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a GameEvent message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns GameEvent
         */
        public static fromObject(object: { [k: string]: any }): proto.GameEvent;

        /**
         * Creates a plain object from a GameEvent message. Also converts values to other types if specified.
         * @param message GameEvent
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.GameEvent, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this GameEvent to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for GameEvent
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace GameEvent {

        /** Properties of a BallLeftField. */
        interface IBallLeftField {

            /** BallLeftField byTeam */
            byTeam: proto.Team;

            /** BallLeftField byBot */
            byBot?: (number|null);

            /** BallLeftField location */
            location?: (proto.IVector2|null);
        }

        /** Represents a BallLeftField. */
        class BallLeftField implements IBallLeftField {

            /**
             * Constructs a new BallLeftField.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBallLeftField);

            /** BallLeftField byTeam. */
            public byTeam: proto.Team;

            /** BallLeftField byBot. */
            public byBot: number;

            /** BallLeftField location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new BallLeftField instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BallLeftField instance
             */
            public static create(properties?: proto.GameEvent.IBallLeftField): proto.GameEvent.BallLeftField;

            /**
             * Encodes the specified BallLeftField message. Does not implicitly {@link proto.GameEvent.BallLeftField.verify|verify} messages.
             * @param message BallLeftField message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBallLeftField, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BallLeftField message, length delimited. Does not implicitly {@link proto.GameEvent.BallLeftField.verify|verify} messages.
             * @param message BallLeftField message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBallLeftField, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BallLeftField message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BallLeftField
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BallLeftField;

            /**
             * Decodes a BallLeftField message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BallLeftField
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BallLeftField;

            /**
             * Verifies a BallLeftField message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BallLeftField message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BallLeftField
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BallLeftField;

            /**
             * Creates a plain object from a BallLeftField message. Also converts values to other types if specified.
             * @param message BallLeftField
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BallLeftField, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BallLeftField to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BallLeftField
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AimlessKick. */
        interface IAimlessKick {

            /** AimlessKick byTeam */
            byTeam: proto.Team;

            /** AimlessKick byBot */
            byBot?: (number|null);

            /** AimlessKick location */
            location?: (proto.IVector2|null);

            /** AimlessKick kickLocation */
            kickLocation?: (proto.IVector2|null);
        }

        /** Represents an AimlessKick. */
        class AimlessKick implements IAimlessKick {

            /**
             * Constructs a new AimlessKick.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAimlessKick);

            /** AimlessKick byTeam. */
            public byTeam: proto.Team;

            /** AimlessKick byBot. */
            public byBot: number;

            /** AimlessKick location. */
            public location?: (proto.IVector2|null);

            /** AimlessKick kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /**
             * Creates a new AimlessKick instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AimlessKick instance
             */
            public static create(properties?: proto.GameEvent.IAimlessKick): proto.GameEvent.AimlessKick;

            /**
             * Encodes the specified AimlessKick message. Does not implicitly {@link proto.GameEvent.AimlessKick.verify|verify} messages.
             * @param message AimlessKick message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAimlessKick, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AimlessKick message, length delimited. Does not implicitly {@link proto.GameEvent.AimlessKick.verify|verify} messages.
             * @param message AimlessKick message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAimlessKick, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AimlessKick message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AimlessKick
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AimlessKick;

            /**
             * Decodes an AimlessKick message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AimlessKick
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AimlessKick;

            /**
             * Verifies an AimlessKick message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AimlessKick message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AimlessKick
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AimlessKick;

            /**
             * Creates a plain object from an AimlessKick message. Also converts values to other types if specified.
             * @param message AimlessKick
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AimlessKick, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AimlessKick to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AimlessKick
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Goal. */
        interface IGoal {

            /** Goal byTeam */
            byTeam: proto.Team;

            /** Goal kickingTeam */
            kickingTeam?: (proto.Team|null);

            /** Goal kickingBot */
            kickingBot?: (number|null);

            /** Goal location */
            location?: (proto.IVector2|null);

            /** Goal kickLocation */
            kickLocation?: (proto.IVector2|null);

            /** Goal maxBallHeight */
            maxBallHeight?: (number|null);

            /** Goal numRobotsByTeam */
            numRobotsByTeam?: (number|null);

            /** Goal lastTouchByTeam */
            lastTouchByTeam?: (number|Long|null);

            /** Goal message */
            message?: (string|null);
        }

        /** Represents a Goal. */
        class Goal implements IGoal {

            /**
             * Constructs a new Goal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IGoal);

            /** Goal byTeam. */
            public byTeam: proto.Team;

            /** Goal kickingTeam. */
            public kickingTeam: proto.Team;

            /** Goal kickingBot. */
            public kickingBot: number;

            /** Goal location. */
            public location?: (proto.IVector2|null);

            /** Goal kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /** Goal maxBallHeight. */
            public maxBallHeight: number;

            /** Goal numRobotsByTeam. */
            public numRobotsByTeam: number;

            /** Goal lastTouchByTeam. */
            public lastTouchByTeam: (number|Long);

            /** Goal message. */
            public message: string;

            /**
             * Creates a new Goal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Goal instance
             */
            public static create(properties?: proto.GameEvent.IGoal): proto.GameEvent.Goal;

            /**
             * Encodes the specified Goal message. Does not implicitly {@link proto.GameEvent.Goal.verify|verify} messages.
             * @param message Goal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Goal message, length delimited. Does not implicitly {@link proto.GameEvent.Goal.verify|verify} messages.
             * @param message Goal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Goal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Goal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.Goal;

            /**
             * Decodes a Goal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Goal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.Goal;

            /**
             * Verifies a Goal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Goal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Goal
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.Goal;

            /**
             * Creates a plain object from a Goal message. Also converts values to other types if specified.
             * @param message Goal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.Goal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Goal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Goal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an IndirectGoal. */
        interface IIndirectGoal {

            /** IndirectGoal byTeam */
            byTeam: proto.Team;

            /** IndirectGoal byBot */
            byBot?: (number|null);

            /** IndirectGoal location */
            location?: (proto.IVector2|null);

            /** IndirectGoal kickLocation */
            kickLocation?: (proto.IVector2|null);
        }

        /** Represents an IndirectGoal. */
        class IndirectGoal implements IIndirectGoal {

            /**
             * Constructs a new IndirectGoal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IIndirectGoal);

            /** IndirectGoal byTeam. */
            public byTeam: proto.Team;

            /** IndirectGoal byBot. */
            public byBot: number;

            /** IndirectGoal location. */
            public location?: (proto.IVector2|null);

            /** IndirectGoal kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /**
             * Creates a new IndirectGoal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns IndirectGoal instance
             */
            public static create(properties?: proto.GameEvent.IIndirectGoal): proto.GameEvent.IndirectGoal;

            /**
             * Encodes the specified IndirectGoal message. Does not implicitly {@link proto.GameEvent.IndirectGoal.verify|verify} messages.
             * @param message IndirectGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IIndirectGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified IndirectGoal message, length delimited. Does not implicitly {@link proto.GameEvent.IndirectGoal.verify|verify} messages.
             * @param message IndirectGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IIndirectGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an IndirectGoal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns IndirectGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.IndirectGoal;

            /**
             * Decodes an IndirectGoal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns IndirectGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.IndirectGoal;

            /**
             * Verifies an IndirectGoal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an IndirectGoal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns IndirectGoal
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.IndirectGoal;

            /**
             * Creates a plain object from an IndirectGoal message. Also converts values to other types if specified.
             * @param message IndirectGoal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.IndirectGoal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this IndirectGoal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for IndirectGoal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a ChippedGoal. */
        interface IChippedGoal {

            /** ChippedGoal byTeam */
            byTeam: proto.Team;

            /** ChippedGoal byBot */
            byBot?: (number|null);

            /** ChippedGoal location */
            location?: (proto.IVector2|null);

            /** ChippedGoal kickLocation */
            kickLocation?: (proto.IVector2|null);

            /** ChippedGoal maxBallHeight */
            maxBallHeight?: (number|null);
        }

        /** Represents a ChippedGoal. */
        class ChippedGoal implements IChippedGoal {

            /**
             * Constructs a new ChippedGoal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IChippedGoal);

            /** ChippedGoal byTeam. */
            public byTeam: proto.Team;

            /** ChippedGoal byBot. */
            public byBot: number;

            /** ChippedGoal location. */
            public location?: (proto.IVector2|null);

            /** ChippedGoal kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /** ChippedGoal maxBallHeight. */
            public maxBallHeight: number;

            /**
             * Creates a new ChippedGoal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns ChippedGoal instance
             */
            public static create(properties?: proto.GameEvent.IChippedGoal): proto.GameEvent.ChippedGoal;

            /**
             * Encodes the specified ChippedGoal message. Does not implicitly {@link proto.GameEvent.ChippedGoal.verify|verify} messages.
             * @param message ChippedGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IChippedGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified ChippedGoal message, length delimited. Does not implicitly {@link proto.GameEvent.ChippedGoal.verify|verify} messages.
             * @param message ChippedGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IChippedGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ChippedGoal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ChippedGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.ChippedGoal;

            /**
             * Decodes a ChippedGoal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns ChippedGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.ChippedGoal;

            /**
             * Verifies a ChippedGoal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a ChippedGoal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns ChippedGoal
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.ChippedGoal;

            /**
             * Creates a plain object from a ChippedGoal message. Also converts values to other types if specified.
             * @param message ChippedGoal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.ChippedGoal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this ChippedGoal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for ChippedGoal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotTooFastInStop. */
        interface IBotTooFastInStop {

            /** BotTooFastInStop byTeam */
            byTeam: proto.Team;

            /** BotTooFastInStop byBot */
            byBot?: (number|null);

            /** BotTooFastInStop location */
            location?: (proto.IVector2|null);

            /** BotTooFastInStop speed */
            speed?: (number|null);
        }

        /** Represents a BotTooFastInStop. */
        class BotTooFastInStop implements IBotTooFastInStop {

            /**
             * Constructs a new BotTooFastInStop.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotTooFastInStop);

            /** BotTooFastInStop byTeam. */
            public byTeam: proto.Team;

            /** BotTooFastInStop byBot. */
            public byBot: number;

            /** BotTooFastInStop location. */
            public location?: (proto.IVector2|null);

            /** BotTooFastInStop speed. */
            public speed: number;

            /**
             * Creates a new BotTooFastInStop instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotTooFastInStop instance
             */
            public static create(properties?: proto.GameEvent.IBotTooFastInStop): proto.GameEvent.BotTooFastInStop;

            /**
             * Encodes the specified BotTooFastInStop message. Does not implicitly {@link proto.GameEvent.BotTooFastInStop.verify|verify} messages.
             * @param message BotTooFastInStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotTooFastInStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotTooFastInStop message, length delimited. Does not implicitly {@link proto.GameEvent.BotTooFastInStop.verify|verify} messages.
             * @param message BotTooFastInStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotTooFastInStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotTooFastInStop message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotTooFastInStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotTooFastInStop;

            /**
             * Decodes a BotTooFastInStop message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotTooFastInStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotTooFastInStop;

            /**
             * Verifies a BotTooFastInStop message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotTooFastInStop message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotTooFastInStop
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotTooFastInStop;

            /**
             * Creates a plain object from a BotTooFastInStop message. Also converts values to other types if specified.
             * @param message BotTooFastInStop
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotTooFastInStop, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotTooFastInStop to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotTooFastInStop
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a DefenderTooCloseToKickPoint. */
        interface IDefenderTooCloseToKickPoint {

            /** DefenderTooCloseToKickPoint byTeam */
            byTeam: proto.Team;

            /** DefenderTooCloseToKickPoint byBot */
            byBot?: (number|null);

            /** DefenderTooCloseToKickPoint location */
            location?: (proto.IVector2|null);

            /** DefenderTooCloseToKickPoint distance */
            distance?: (number|null);
        }

        /** Represents a DefenderTooCloseToKickPoint. */
        class DefenderTooCloseToKickPoint implements IDefenderTooCloseToKickPoint {

            /**
             * Constructs a new DefenderTooCloseToKickPoint.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IDefenderTooCloseToKickPoint);

            /** DefenderTooCloseToKickPoint byTeam. */
            public byTeam: proto.Team;

            /** DefenderTooCloseToKickPoint byBot. */
            public byBot: number;

            /** DefenderTooCloseToKickPoint location. */
            public location?: (proto.IVector2|null);

            /** DefenderTooCloseToKickPoint distance. */
            public distance: number;

            /**
             * Creates a new DefenderTooCloseToKickPoint instance using the specified properties.
             * @param [properties] Properties to set
             * @returns DefenderTooCloseToKickPoint instance
             */
            public static create(properties?: proto.GameEvent.IDefenderTooCloseToKickPoint): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Encodes the specified DefenderTooCloseToKickPoint message. Does not implicitly {@link proto.GameEvent.DefenderTooCloseToKickPoint.verify|verify} messages.
             * @param message DefenderTooCloseToKickPoint message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IDefenderTooCloseToKickPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified DefenderTooCloseToKickPoint message, length delimited. Does not implicitly {@link proto.GameEvent.DefenderTooCloseToKickPoint.verify|verify} messages.
             * @param message DefenderTooCloseToKickPoint message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IDefenderTooCloseToKickPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DefenderTooCloseToKickPoint message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DefenderTooCloseToKickPoint
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Decodes a DefenderTooCloseToKickPoint message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns DefenderTooCloseToKickPoint
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Verifies a DefenderTooCloseToKickPoint message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a DefenderTooCloseToKickPoint message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns DefenderTooCloseToKickPoint
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Creates a plain object from a DefenderTooCloseToKickPoint message. Also converts values to other types if specified.
             * @param message DefenderTooCloseToKickPoint
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.DefenderTooCloseToKickPoint, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this DefenderTooCloseToKickPoint to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for DefenderTooCloseToKickPoint
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotCrashDrawn. */
        interface IBotCrashDrawn {

            /** BotCrashDrawn botYellow */
            botYellow?: (number|null);

            /** BotCrashDrawn botBlue */
            botBlue?: (number|null);

            /** BotCrashDrawn location */
            location?: (proto.IVector2|null);

            /** BotCrashDrawn crashSpeed */
            crashSpeed?: (number|null);

            /** BotCrashDrawn speedDiff */
            speedDiff?: (number|null);

            /** BotCrashDrawn crashAngle */
            crashAngle?: (number|null);
        }

        /** Represents a BotCrashDrawn. */
        class BotCrashDrawn implements IBotCrashDrawn {

            /**
             * Constructs a new BotCrashDrawn.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotCrashDrawn);

            /** BotCrashDrawn botYellow. */
            public botYellow: number;

            /** BotCrashDrawn botBlue. */
            public botBlue: number;

            /** BotCrashDrawn location. */
            public location?: (proto.IVector2|null);

            /** BotCrashDrawn crashSpeed. */
            public crashSpeed: number;

            /** BotCrashDrawn speedDiff. */
            public speedDiff: number;

            /** BotCrashDrawn crashAngle. */
            public crashAngle: number;

            /**
             * Creates a new BotCrashDrawn instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotCrashDrawn instance
             */
            public static create(properties?: proto.GameEvent.IBotCrashDrawn): proto.GameEvent.BotCrashDrawn;

            /**
             * Encodes the specified BotCrashDrawn message. Does not implicitly {@link proto.GameEvent.BotCrashDrawn.verify|verify} messages.
             * @param message BotCrashDrawn message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotCrashDrawn, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotCrashDrawn message, length delimited. Does not implicitly {@link proto.GameEvent.BotCrashDrawn.verify|verify} messages.
             * @param message BotCrashDrawn message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotCrashDrawn, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotCrashDrawn message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotCrashDrawn
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotCrashDrawn;

            /**
             * Decodes a BotCrashDrawn message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotCrashDrawn
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotCrashDrawn;

            /**
             * Verifies a BotCrashDrawn message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotCrashDrawn message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotCrashDrawn
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotCrashDrawn;

            /**
             * Creates a plain object from a BotCrashDrawn message. Also converts values to other types if specified.
             * @param message BotCrashDrawn
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotCrashDrawn, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotCrashDrawn to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotCrashDrawn
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotCrashUnique. */
        interface IBotCrashUnique {

            /** BotCrashUnique byTeam */
            byTeam: proto.Team;

            /** BotCrashUnique violator */
            violator?: (number|null);

            /** BotCrashUnique victim */
            victim?: (number|null);

            /** BotCrashUnique location */
            location?: (proto.IVector2|null);

            /** BotCrashUnique crashSpeed */
            crashSpeed?: (number|null);

            /** BotCrashUnique speedDiff */
            speedDiff?: (number|null);

            /** BotCrashUnique crashAngle */
            crashAngle?: (number|null);
        }

        /** Represents a BotCrashUnique. */
        class BotCrashUnique implements IBotCrashUnique {

            /**
             * Constructs a new BotCrashUnique.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotCrashUnique);

            /** BotCrashUnique byTeam. */
            public byTeam: proto.Team;

            /** BotCrashUnique violator. */
            public violator: number;

            /** BotCrashUnique victim. */
            public victim: number;

            /** BotCrashUnique location. */
            public location?: (proto.IVector2|null);

            /** BotCrashUnique crashSpeed. */
            public crashSpeed: number;

            /** BotCrashUnique speedDiff. */
            public speedDiff: number;

            /** BotCrashUnique crashAngle. */
            public crashAngle: number;

            /**
             * Creates a new BotCrashUnique instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotCrashUnique instance
             */
            public static create(properties?: proto.GameEvent.IBotCrashUnique): proto.GameEvent.BotCrashUnique;

            /**
             * Encodes the specified BotCrashUnique message. Does not implicitly {@link proto.GameEvent.BotCrashUnique.verify|verify} messages.
             * @param message BotCrashUnique message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotCrashUnique, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotCrashUnique message, length delimited. Does not implicitly {@link proto.GameEvent.BotCrashUnique.verify|verify} messages.
             * @param message BotCrashUnique message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotCrashUnique, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotCrashUnique message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotCrashUnique
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotCrashUnique;

            /**
             * Decodes a BotCrashUnique message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotCrashUnique
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotCrashUnique;

            /**
             * Verifies a BotCrashUnique message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotCrashUnique message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotCrashUnique
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotCrashUnique;

            /**
             * Creates a plain object from a BotCrashUnique message. Also converts values to other types if specified.
             * @param message BotCrashUnique
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotCrashUnique, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotCrashUnique to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotCrashUnique
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotPushedBot. */
        interface IBotPushedBot {

            /** BotPushedBot byTeam */
            byTeam: proto.Team;

            /** BotPushedBot violator */
            violator?: (number|null);

            /** BotPushedBot victim */
            victim?: (number|null);

            /** BotPushedBot location */
            location?: (proto.IVector2|null);

            /** BotPushedBot pushedDistance */
            pushedDistance?: (number|null);
        }

        /** Represents a BotPushedBot. */
        class BotPushedBot implements IBotPushedBot {

            /**
             * Constructs a new BotPushedBot.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotPushedBot);

            /** BotPushedBot byTeam. */
            public byTeam: proto.Team;

            /** BotPushedBot violator. */
            public violator: number;

            /** BotPushedBot victim. */
            public victim: number;

            /** BotPushedBot location. */
            public location?: (proto.IVector2|null);

            /** BotPushedBot pushedDistance. */
            public pushedDistance: number;

            /**
             * Creates a new BotPushedBot instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotPushedBot instance
             */
            public static create(properties?: proto.GameEvent.IBotPushedBot): proto.GameEvent.BotPushedBot;

            /**
             * Encodes the specified BotPushedBot message. Does not implicitly {@link proto.GameEvent.BotPushedBot.verify|verify} messages.
             * @param message BotPushedBot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotPushedBot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotPushedBot message, length delimited. Does not implicitly {@link proto.GameEvent.BotPushedBot.verify|verify} messages.
             * @param message BotPushedBot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotPushedBot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotPushedBot message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotPushedBot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotPushedBot;

            /**
             * Decodes a BotPushedBot message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotPushedBot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotPushedBot;

            /**
             * Verifies a BotPushedBot message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotPushedBot message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotPushedBot
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotPushedBot;

            /**
             * Creates a plain object from a BotPushedBot message. Also converts values to other types if specified.
             * @param message BotPushedBot
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotPushedBot, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotPushedBot to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotPushedBot
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotTippedOver. */
        interface IBotTippedOver {

            /** BotTippedOver byTeam */
            byTeam: proto.Team;

            /** BotTippedOver byBot */
            byBot?: (number|null);

            /** BotTippedOver location */
            location?: (proto.IVector2|null);

            /** BotTippedOver ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents a BotTippedOver. */
        class BotTippedOver implements IBotTippedOver {

            /**
             * Constructs a new BotTippedOver.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotTippedOver);

            /** BotTippedOver byTeam. */
            public byTeam: proto.Team;

            /** BotTippedOver byBot. */
            public byBot: number;

            /** BotTippedOver location. */
            public location?: (proto.IVector2|null);

            /** BotTippedOver ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new BotTippedOver instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotTippedOver instance
             */
            public static create(properties?: proto.GameEvent.IBotTippedOver): proto.GameEvent.BotTippedOver;

            /**
             * Encodes the specified BotTippedOver message. Does not implicitly {@link proto.GameEvent.BotTippedOver.verify|verify} messages.
             * @param message BotTippedOver message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotTippedOver, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotTippedOver message, length delimited. Does not implicitly {@link proto.GameEvent.BotTippedOver.verify|verify} messages.
             * @param message BotTippedOver message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotTippedOver, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotTippedOver message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotTippedOver
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotTippedOver;

            /**
             * Decodes a BotTippedOver message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotTippedOver
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotTippedOver;

            /**
             * Verifies a BotTippedOver message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotTippedOver message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotTippedOver
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotTippedOver;

            /**
             * Creates a plain object from a BotTippedOver message. Also converts values to other types if specified.
             * @param message BotTippedOver
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotTippedOver, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotTippedOver to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotTippedOver
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a DefenderInDefenseArea. */
        interface IDefenderInDefenseArea {

            /** DefenderInDefenseArea byTeam */
            byTeam: proto.Team;

            /** DefenderInDefenseArea byBot */
            byBot?: (number|null);

            /** DefenderInDefenseArea location */
            location?: (proto.IVector2|null);

            /** DefenderInDefenseArea distance */
            distance?: (number|null);
        }

        /** Represents a DefenderInDefenseArea. */
        class DefenderInDefenseArea implements IDefenderInDefenseArea {

            /**
             * Constructs a new DefenderInDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IDefenderInDefenseArea);

            /** DefenderInDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** DefenderInDefenseArea byBot. */
            public byBot: number;

            /** DefenderInDefenseArea location. */
            public location?: (proto.IVector2|null);

            /** DefenderInDefenseArea distance. */
            public distance: number;

            /**
             * Creates a new DefenderInDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns DefenderInDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IDefenderInDefenseArea): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Encodes the specified DefenderInDefenseArea message. Does not implicitly {@link proto.GameEvent.DefenderInDefenseArea.verify|verify} messages.
             * @param message DefenderInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IDefenderInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified DefenderInDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.DefenderInDefenseArea.verify|verify} messages.
             * @param message DefenderInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IDefenderInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DefenderInDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DefenderInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Decodes a DefenderInDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns DefenderInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Verifies a DefenderInDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a DefenderInDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns DefenderInDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Creates a plain object from a DefenderInDefenseArea message. Also converts values to other types if specified.
             * @param message DefenderInDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.DefenderInDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this DefenderInDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for DefenderInDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a DefenderInDefenseAreaPartially. */
        interface IDefenderInDefenseAreaPartially {

            /** DefenderInDefenseAreaPartially byTeam */
            byTeam: proto.Team;

            /** DefenderInDefenseAreaPartially byBot */
            byBot?: (number|null);

            /** DefenderInDefenseAreaPartially location */
            location?: (proto.IVector2|null);

            /** DefenderInDefenseAreaPartially distance */
            distance?: (number|null);

            /** DefenderInDefenseAreaPartially ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents a DefenderInDefenseAreaPartially. */
        class DefenderInDefenseAreaPartially implements IDefenderInDefenseAreaPartially {

            /**
             * Constructs a new DefenderInDefenseAreaPartially.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IDefenderInDefenseAreaPartially);

            /** DefenderInDefenseAreaPartially byTeam. */
            public byTeam: proto.Team;

            /** DefenderInDefenseAreaPartially byBot. */
            public byBot: number;

            /** DefenderInDefenseAreaPartially location. */
            public location?: (proto.IVector2|null);

            /** DefenderInDefenseAreaPartially distance. */
            public distance: number;

            /** DefenderInDefenseAreaPartially ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new DefenderInDefenseAreaPartially instance using the specified properties.
             * @param [properties] Properties to set
             * @returns DefenderInDefenseAreaPartially instance
             */
            public static create(properties?: proto.GameEvent.IDefenderInDefenseAreaPartially): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Encodes the specified DefenderInDefenseAreaPartially message. Does not implicitly {@link proto.GameEvent.DefenderInDefenseAreaPartially.verify|verify} messages.
             * @param message DefenderInDefenseAreaPartially message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IDefenderInDefenseAreaPartially, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified DefenderInDefenseAreaPartially message, length delimited. Does not implicitly {@link proto.GameEvent.DefenderInDefenseAreaPartially.verify|verify} messages.
             * @param message DefenderInDefenseAreaPartially message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IDefenderInDefenseAreaPartially, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DefenderInDefenseAreaPartially message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DefenderInDefenseAreaPartially
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Decodes a DefenderInDefenseAreaPartially message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns DefenderInDefenseAreaPartially
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Verifies a DefenderInDefenseAreaPartially message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a DefenderInDefenseAreaPartially message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns DefenderInDefenseAreaPartially
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Creates a plain object from a DefenderInDefenseAreaPartially message. Also converts values to other types if specified.
             * @param message DefenderInDefenseAreaPartially
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.DefenderInDefenseAreaPartially, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this DefenderInDefenseAreaPartially to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for DefenderInDefenseAreaPartially
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerTouchedBallInDefenseArea. */
        interface IAttackerTouchedBallInDefenseArea {

            /** AttackerTouchedBallInDefenseArea byTeam */
            byTeam: proto.Team;

            /** AttackerTouchedBallInDefenseArea byBot */
            byBot?: (number|null);

            /** AttackerTouchedBallInDefenseArea location */
            location?: (proto.IVector2|null);

            /** AttackerTouchedBallInDefenseArea distance */
            distance?: (number|null);
        }

        /** Represents an AttackerTouchedBallInDefenseArea. */
        class AttackerTouchedBallInDefenseArea implements IAttackerTouchedBallInDefenseArea {

            /**
             * Constructs a new AttackerTouchedBallInDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerTouchedBallInDefenseArea);

            /** AttackerTouchedBallInDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** AttackerTouchedBallInDefenseArea byBot. */
            public byBot: number;

            /** AttackerTouchedBallInDefenseArea location. */
            public location?: (proto.IVector2|null);

            /** AttackerTouchedBallInDefenseArea distance. */
            public distance: number;

            /**
             * Creates a new AttackerTouchedBallInDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerTouchedBallInDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IAttackerTouchedBallInDefenseArea): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Encodes the specified AttackerTouchedBallInDefenseArea message. Does not implicitly {@link proto.GameEvent.AttackerTouchedBallInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedBallInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerTouchedBallInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerTouchedBallInDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerTouchedBallInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedBallInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerTouchedBallInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerTouchedBallInDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerTouchedBallInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Decodes an AttackerTouchedBallInDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerTouchedBallInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Verifies an AttackerTouchedBallInDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerTouchedBallInDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerTouchedBallInDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Creates a plain object from an AttackerTouchedBallInDefenseArea message. Also converts values to other types if specified.
             * @param message AttackerTouchedBallInDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerTouchedBallInDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerTouchedBallInDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerTouchedBallInDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotKickedBallTooFast. */
        interface IBotKickedBallTooFast {

            /** BotKickedBallTooFast byTeam */
            byTeam: proto.Team;

            /** BotKickedBallTooFast byBot */
            byBot?: (number|null);

            /** BotKickedBallTooFast location */
            location?: (proto.IVector2|null);

            /** BotKickedBallTooFast initialBallSpeed */
            initialBallSpeed?: (number|null);

            /** BotKickedBallTooFast chipped */
            chipped?: (boolean|null);
        }

        /** Represents a BotKickedBallTooFast. */
        class BotKickedBallTooFast implements IBotKickedBallTooFast {

            /**
             * Constructs a new BotKickedBallTooFast.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotKickedBallTooFast);

            /** BotKickedBallTooFast byTeam. */
            public byTeam: proto.Team;

            /** BotKickedBallTooFast byBot. */
            public byBot: number;

            /** BotKickedBallTooFast location. */
            public location?: (proto.IVector2|null);

            /** BotKickedBallTooFast initialBallSpeed. */
            public initialBallSpeed: number;

            /** BotKickedBallTooFast chipped. */
            public chipped: boolean;

            /**
             * Creates a new BotKickedBallTooFast instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotKickedBallTooFast instance
             */
            public static create(properties?: proto.GameEvent.IBotKickedBallTooFast): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Encodes the specified BotKickedBallTooFast message. Does not implicitly {@link proto.GameEvent.BotKickedBallTooFast.verify|verify} messages.
             * @param message BotKickedBallTooFast message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotKickedBallTooFast, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotKickedBallTooFast message, length delimited. Does not implicitly {@link proto.GameEvent.BotKickedBallTooFast.verify|verify} messages.
             * @param message BotKickedBallTooFast message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotKickedBallTooFast, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotKickedBallTooFast message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotKickedBallTooFast
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Decodes a BotKickedBallTooFast message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotKickedBallTooFast
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Verifies a BotKickedBallTooFast message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotKickedBallTooFast message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotKickedBallTooFast
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Creates a plain object from a BotKickedBallTooFast message. Also converts values to other types if specified.
             * @param message BotKickedBallTooFast
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotKickedBallTooFast, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotKickedBallTooFast to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotKickedBallTooFast
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotDribbledBallTooFar. */
        interface IBotDribbledBallTooFar {

            /** BotDribbledBallTooFar byTeam */
            byTeam: proto.Team;

            /** BotDribbledBallTooFar byBot */
            byBot?: (number|null);

            /** BotDribbledBallTooFar start */
            start?: (proto.IVector2|null);

            /** BotDribbledBallTooFar end */
            end?: (proto.IVector2|null);
        }

        /** Represents a BotDribbledBallTooFar. */
        class BotDribbledBallTooFar implements IBotDribbledBallTooFar {

            /**
             * Constructs a new BotDribbledBallTooFar.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotDribbledBallTooFar);

            /** BotDribbledBallTooFar byTeam. */
            public byTeam: proto.Team;

            /** BotDribbledBallTooFar byBot. */
            public byBot: number;

            /** BotDribbledBallTooFar start. */
            public start?: (proto.IVector2|null);

            /** BotDribbledBallTooFar end. */
            public end?: (proto.IVector2|null);

            /**
             * Creates a new BotDribbledBallTooFar instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotDribbledBallTooFar instance
             */
            public static create(properties?: proto.GameEvent.IBotDribbledBallTooFar): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Encodes the specified BotDribbledBallTooFar message. Does not implicitly {@link proto.GameEvent.BotDribbledBallTooFar.verify|verify} messages.
             * @param message BotDribbledBallTooFar message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotDribbledBallTooFar, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotDribbledBallTooFar message, length delimited. Does not implicitly {@link proto.GameEvent.BotDribbledBallTooFar.verify|verify} messages.
             * @param message BotDribbledBallTooFar message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotDribbledBallTooFar, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotDribbledBallTooFar message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotDribbledBallTooFar
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Decodes a BotDribbledBallTooFar message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotDribbledBallTooFar
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Verifies a BotDribbledBallTooFar message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotDribbledBallTooFar message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotDribbledBallTooFar
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Creates a plain object from a BotDribbledBallTooFar message. Also converts values to other types if specified.
             * @param message BotDribbledBallTooFar
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotDribbledBallTooFar, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotDribbledBallTooFar to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotDribbledBallTooFar
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerTouchedOpponentInDefenseArea. */
        interface IAttackerTouchedOpponentInDefenseArea {

            /** AttackerTouchedOpponentInDefenseArea byTeam */
            byTeam: proto.Team;

            /** AttackerTouchedOpponentInDefenseArea byBot */
            byBot?: (number|null);

            /** AttackerTouchedOpponentInDefenseArea victim */
            victim?: (number|null);

            /** AttackerTouchedOpponentInDefenseArea location */
            location?: (proto.IVector2|null);
        }

        /** Represents an AttackerTouchedOpponentInDefenseArea. */
        class AttackerTouchedOpponentInDefenseArea implements IAttackerTouchedOpponentInDefenseArea {

            /**
             * Constructs a new AttackerTouchedOpponentInDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea);

            /** AttackerTouchedOpponentInDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** AttackerTouchedOpponentInDefenseArea byBot. */
            public byBot: number;

            /** AttackerTouchedOpponentInDefenseArea victim. */
            public victim: number;

            /** AttackerTouchedOpponentInDefenseArea location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new AttackerTouchedOpponentInDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerTouchedOpponentInDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Encodes the specified AttackerTouchedOpponentInDefenseArea message. Does not implicitly {@link proto.GameEvent.AttackerTouchedOpponentInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedOpponentInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerTouchedOpponentInDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerTouchedOpponentInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedOpponentInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerTouchedOpponentInDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerTouchedOpponentInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Decodes an AttackerTouchedOpponentInDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerTouchedOpponentInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Verifies an AttackerTouchedOpponentInDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerTouchedOpponentInDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerTouchedOpponentInDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Creates a plain object from an AttackerTouchedOpponentInDefenseArea message. Also converts values to other types if specified.
             * @param message AttackerTouchedOpponentInDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerTouchedOpponentInDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerTouchedOpponentInDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerTouchedOpponentInDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerDoubleTouchedBall. */
        interface IAttackerDoubleTouchedBall {

            /** AttackerDoubleTouchedBall byTeam */
            byTeam: proto.Team;

            /** AttackerDoubleTouchedBall byBot */
            byBot?: (number|null);

            /** AttackerDoubleTouchedBall location */
            location?: (proto.IVector2|null);
        }

        /** Represents an AttackerDoubleTouchedBall. */
        class AttackerDoubleTouchedBall implements IAttackerDoubleTouchedBall {

            /**
             * Constructs a new AttackerDoubleTouchedBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerDoubleTouchedBall);

            /** AttackerDoubleTouchedBall byTeam. */
            public byTeam: proto.Team;

            /** AttackerDoubleTouchedBall byBot. */
            public byBot: number;

            /** AttackerDoubleTouchedBall location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new AttackerDoubleTouchedBall instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerDoubleTouchedBall instance
             */
            public static create(properties?: proto.GameEvent.IAttackerDoubleTouchedBall): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Encodes the specified AttackerDoubleTouchedBall message. Does not implicitly {@link proto.GameEvent.AttackerDoubleTouchedBall.verify|verify} messages.
             * @param message AttackerDoubleTouchedBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerDoubleTouchedBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerDoubleTouchedBall message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerDoubleTouchedBall.verify|verify} messages.
             * @param message AttackerDoubleTouchedBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerDoubleTouchedBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerDoubleTouchedBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerDoubleTouchedBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Decodes an AttackerDoubleTouchedBall message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerDoubleTouchedBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Verifies an AttackerDoubleTouchedBall message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerDoubleTouchedBall message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerDoubleTouchedBall
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Creates a plain object from an AttackerDoubleTouchedBall message. Also converts values to other types if specified.
             * @param message AttackerDoubleTouchedBall
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerDoubleTouchedBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerDoubleTouchedBall to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerDoubleTouchedBall
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerTooCloseToDefenseArea. */
        interface IAttackerTooCloseToDefenseArea {

            /** AttackerTooCloseToDefenseArea byTeam */
            byTeam: proto.Team;

            /** AttackerTooCloseToDefenseArea byBot */
            byBot?: (number|null);

            /** AttackerTooCloseToDefenseArea location */
            location?: (proto.IVector2|null);

            /** AttackerTooCloseToDefenseArea distance */
            distance?: (number|null);

            /** AttackerTooCloseToDefenseArea ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents an AttackerTooCloseToDefenseArea. */
        class AttackerTooCloseToDefenseArea implements IAttackerTooCloseToDefenseArea {

            /**
             * Constructs a new AttackerTooCloseToDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerTooCloseToDefenseArea);

            /** AttackerTooCloseToDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** AttackerTooCloseToDefenseArea byBot. */
            public byBot: number;

            /** AttackerTooCloseToDefenseArea location. */
            public location?: (proto.IVector2|null);

            /** AttackerTooCloseToDefenseArea distance. */
            public distance: number;

            /** AttackerTooCloseToDefenseArea ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new AttackerTooCloseToDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerTooCloseToDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IAttackerTooCloseToDefenseArea): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Encodes the specified AttackerTooCloseToDefenseArea message. Does not implicitly {@link proto.GameEvent.AttackerTooCloseToDefenseArea.verify|verify} messages.
             * @param message AttackerTooCloseToDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerTooCloseToDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerTooCloseToDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerTooCloseToDefenseArea.verify|verify} messages.
             * @param message AttackerTooCloseToDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerTooCloseToDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerTooCloseToDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerTooCloseToDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Decodes an AttackerTooCloseToDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerTooCloseToDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Verifies an AttackerTooCloseToDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerTooCloseToDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerTooCloseToDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Creates a plain object from an AttackerTooCloseToDefenseArea message. Also converts values to other types if specified.
             * @param message AttackerTooCloseToDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerTooCloseToDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerTooCloseToDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerTooCloseToDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotHeldBallDeliberately. */
        interface IBotHeldBallDeliberately {

            /** BotHeldBallDeliberately byTeam */
            byTeam: proto.Team;

            /** BotHeldBallDeliberately byBot */
            byBot?: (number|null);

            /** BotHeldBallDeliberately location */
            location?: (proto.IVector2|null);

            /** BotHeldBallDeliberately duration */
            duration?: (number|null);
        }

        /** Represents a BotHeldBallDeliberately. */
        class BotHeldBallDeliberately implements IBotHeldBallDeliberately {

            /**
             * Constructs a new BotHeldBallDeliberately.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotHeldBallDeliberately);

            /** BotHeldBallDeliberately byTeam. */
            public byTeam: proto.Team;

            /** BotHeldBallDeliberately byBot. */
            public byBot: number;

            /** BotHeldBallDeliberately location. */
            public location?: (proto.IVector2|null);

            /** BotHeldBallDeliberately duration. */
            public duration: number;

            /**
             * Creates a new BotHeldBallDeliberately instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotHeldBallDeliberately instance
             */
            public static create(properties?: proto.GameEvent.IBotHeldBallDeliberately): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Encodes the specified BotHeldBallDeliberately message. Does not implicitly {@link proto.GameEvent.BotHeldBallDeliberately.verify|verify} messages.
             * @param message BotHeldBallDeliberately message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotHeldBallDeliberately, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotHeldBallDeliberately message, length delimited. Does not implicitly {@link proto.GameEvent.BotHeldBallDeliberately.verify|verify} messages.
             * @param message BotHeldBallDeliberately message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotHeldBallDeliberately, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotHeldBallDeliberately message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotHeldBallDeliberately
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Decodes a BotHeldBallDeliberately message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotHeldBallDeliberately
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Verifies a BotHeldBallDeliberately message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotHeldBallDeliberately message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotHeldBallDeliberately
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Creates a plain object from a BotHeldBallDeliberately message. Also converts values to other types if specified.
             * @param message BotHeldBallDeliberately
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotHeldBallDeliberately, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotHeldBallDeliberately to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotHeldBallDeliberately
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotInterferedPlacement. */
        interface IBotInterferedPlacement {

            /** BotInterferedPlacement byTeam */
            byTeam: proto.Team;

            /** BotInterferedPlacement byBot */
            byBot?: (number|null);

            /** BotInterferedPlacement location */
            location?: (proto.IVector2|null);
        }

        /** Represents a BotInterferedPlacement. */
        class BotInterferedPlacement implements IBotInterferedPlacement {

            /**
             * Constructs a new BotInterferedPlacement.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotInterferedPlacement);

            /** BotInterferedPlacement byTeam. */
            public byTeam: proto.Team;

            /** BotInterferedPlacement byBot. */
            public byBot: number;

            /** BotInterferedPlacement location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new BotInterferedPlacement instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotInterferedPlacement instance
             */
            public static create(properties?: proto.GameEvent.IBotInterferedPlacement): proto.GameEvent.BotInterferedPlacement;

            /**
             * Encodes the specified BotInterferedPlacement message. Does not implicitly {@link proto.GameEvent.BotInterferedPlacement.verify|verify} messages.
             * @param message BotInterferedPlacement message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotInterferedPlacement, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotInterferedPlacement message, length delimited. Does not implicitly {@link proto.GameEvent.BotInterferedPlacement.verify|verify} messages.
             * @param message BotInterferedPlacement message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotInterferedPlacement, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotInterferedPlacement message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotInterferedPlacement
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotInterferedPlacement;

            /**
             * Decodes a BotInterferedPlacement message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotInterferedPlacement
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotInterferedPlacement;

            /**
             * Verifies a BotInterferedPlacement message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotInterferedPlacement message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotInterferedPlacement
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotInterferedPlacement;

            /**
             * Creates a plain object from a BotInterferedPlacement message. Also converts values to other types if specified.
             * @param message BotInterferedPlacement
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotInterferedPlacement, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotInterferedPlacement to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotInterferedPlacement
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a MultipleCards. */
        interface IMultipleCards {

            /** MultipleCards byTeam */
            byTeam: proto.Team;
        }

        /** Represents a MultipleCards. */
        class MultipleCards implements IMultipleCards {

            /**
             * Constructs a new MultipleCards.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IMultipleCards);

            /** MultipleCards byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new MultipleCards instance using the specified properties.
             * @param [properties] Properties to set
             * @returns MultipleCards instance
             */
            public static create(properties?: proto.GameEvent.IMultipleCards): proto.GameEvent.MultipleCards;

            /**
             * Encodes the specified MultipleCards message. Does not implicitly {@link proto.GameEvent.MultipleCards.verify|verify} messages.
             * @param message MultipleCards message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IMultipleCards, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified MultipleCards message, length delimited. Does not implicitly {@link proto.GameEvent.MultipleCards.verify|verify} messages.
             * @param message MultipleCards message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IMultipleCards, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MultipleCards message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MultipleCards
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.MultipleCards;

            /**
             * Decodes a MultipleCards message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns MultipleCards
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.MultipleCards;

            /**
             * Verifies a MultipleCards message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a MultipleCards message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns MultipleCards
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.MultipleCards;

            /**
             * Creates a plain object from a MultipleCards message. Also converts values to other types if specified.
             * @param message MultipleCards
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.MultipleCards, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this MultipleCards to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for MultipleCards
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a MultipleFouls. */
        interface IMultipleFouls {

            /** MultipleFouls byTeam */
            byTeam: proto.Team;
        }

        /** Represents a MultipleFouls. */
        class MultipleFouls implements IMultipleFouls {

            /**
             * Constructs a new MultipleFouls.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IMultipleFouls);

            /** MultipleFouls byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new MultipleFouls instance using the specified properties.
             * @param [properties] Properties to set
             * @returns MultipleFouls instance
             */
            public static create(properties?: proto.GameEvent.IMultipleFouls): proto.GameEvent.MultipleFouls;

            /**
             * Encodes the specified MultipleFouls message. Does not implicitly {@link proto.GameEvent.MultipleFouls.verify|verify} messages.
             * @param message MultipleFouls message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IMultipleFouls, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified MultipleFouls message, length delimited. Does not implicitly {@link proto.GameEvent.MultipleFouls.verify|verify} messages.
             * @param message MultipleFouls message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IMultipleFouls, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MultipleFouls message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MultipleFouls
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.MultipleFouls;

            /**
             * Decodes a MultipleFouls message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns MultipleFouls
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.MultipleFouls;

            /**
             * Verifies a MultipleFouls message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a MultipleFouls message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns MultipleFouls
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.MultipleFouls;

            /**
             * Creates a plain object from a MultipleFouls message. Also converts values to other types if specified.
             * @param message MultipleFouls
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.MultipleFouls, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this MultipleFouls to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for MultipleFouls
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a MultiplePlacementFailures. */
        interface IMultiplePlacementFailures {

            /** MultiplePlacementFailures byTeam */
            byTeam: proto.Team;
        }

        /** Represents a MultiplePlacementFailures. */
        class MultiplePlacementFailures implements IMultiplePlacementFailures {

            /**
             * Constructs a new MultiplePlacementFailures.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IMultiplePlacementFailures);

            /** MultiplePlacementFailures byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new MultiplePlacementFailures instance using the specified properties.
             * @param [properties] Properties to set
             * @returns MultiplePlacementFailures instance
             */
            public static create(properties?: proto.GameEvent.IMultiplePlacementFailures): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Encodes the specified MultiplePlacementFailures message. Does not implicitly {@link proto.GameEvent.MultiplePlacementFailures.verify|verify} messages.
             * @param message MultiplePlacementFailures message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IMultiplePlacementFailures, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified MultiplePlacementFailures message, length delimited. Does not implicitly {@link proto.GameEvent.MultiplePlacementFailures.verify|verify} messages.
             * @param message MultiplePlacementFailures message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IMultiplePlacementFailures, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MultiplePlacementFailures message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MultiplePlacementFailures
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Decodes a MultiplePlacementFailures message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns MultiplePlacementFailures
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Verifies a MultiplePlacementFailures message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a MultiplePlacementFailures message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns MultiplePlacementFailures
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Creates a plain object from a MultiplePlacementFailures message. Also converts values to other types if specified.
             * @param message MultiplePlacementFailures
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.MultiplePlacementFailures, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this MultiplePlacementFailures to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for MultiplePlacementFailures
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a KickTimeout. */
        interface IKickTimeout {

            /** KickTimeout byTeam */
            byTeam: proto.Team;

            /** KickTimeout location */
            location?: (proto.IVector2|null);

            /** KickTimeout time */
            time?: (number|null);
        }

        /** Represents a KickTimeout. */
        class KickTimeout implements IKickTimeout {

            /**
             * Constructs a new KickTimeout.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IKickTimeout);

            /** KickTimeout byTeam. */
            public byTeam: proto.Team;

            /** KickTimeout location. */
            public location?: (proto.IVector2|null);

            /** KickTimeout time. */
            public time: number;

            /**
             * Creates a new KickTimeout instance using the specified properties.
             * @param [properties] Properties to set
             * @returns KickTimeout instance
             */
            public static create(properties?: proto.GameEvent.IKickTimeout): proto.GameEvent.KickTimeout;

            /**
             * Encodes the specified KickTimeout message. Does not implicitly {@link proto.GameEvent.KickTimeout.verify|verify} messages.
             * @param message KickTimeout message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IKickTimeout, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified KickTimeout message, length delimited. Does not implicitly {@link proto.GameEvent.KickTimeout.verify|verify} messages.
             * @param message KickTimeout message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IKickTimeout, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KickTimeout message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KickTimeout
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.KickTimeout;

            /**
             * Decodes a KickTimeout message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns KickTimeout
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.KickTimeout;

            /**
             * Verifies a KickTimeout message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a KickTimeout message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns KickTimeout
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.KickTimeout;

            /**
             * Creates a plain object from a KickTimeout message. Also converts values to other types if specified.
             * @param message KickTimeout
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.KickTimeout, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this KickTimeout to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for KickTimeout
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a NoProgressInGame. */
        interface INoProgressInGame {

            /** NoProgressInGame location */
            location?: (proto.IVector2|null);

            /** NoProgressInGame time */
            time?: (number|null);
        }

        /** Represents a NoProgressInGame. */
        class NoProgressInGame implements INoProgressInGame {

            /**
             * Constructs a new NoProgressInGame.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.INoProgressInGame);

            /** NoProgressInGame location. */
            public location?: (proto.IVector2|null);

            /** NoProgressInGame time. */
            public time: number;

            /**
             * Creates a new NoProgressInGame instance using the specified properties.
             * @param [properties] Properties to set
             * @returns NoProgressInGame instance
             */
            public static create(properties?: proto.GameEvent.INoProgressInGame): proto.GameEvent.NoProgressInGame;

            /**
             * Encodes the specified NoProgressInGame message. Does not implicitly {@link proto.GameEvent.NoProgressInGame.verify|verify} messages.
             * @param message NoProgressInGame message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.INoProgressInGame, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified NoProgressInGame message, length delimited. Does not implicitly {@link proto.GameEvent.NoProgressInGame.verify|verify} messages.
             * @param message NoProgressInGame message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.INoProgressInGame, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a NoProgressInGame message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns NoProgressInGame
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.NoProgressInGame;

            /**
             * Decodes a NoProgressInGame message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns NoProgressInGame
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.NoProgressInGame;

            /**
             * Verifies a NoProgressInGame message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a NoProgressInGame message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns NoProgressInGame
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.NoProgressInGame;

            /**
             * Creates a plain object from a NoProgressInGame message. Also converts values to other types if specified.
             * @param message NoProgressInGame
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.NoProgressInGame, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this NoProgressInGame to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for NoProgressInGame
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a PlacementFailed. */
        interface IPlacementFailed {

            /** PlacementFailed byTeam */
            byTeam: proto.Team;

            /** PlacementFailed remainingDistance */
            remainingDistance?: (number|null);
        }

        /** Represents a PlacementFailed. */
        class PlacementFailed implements IPlacementFailed {

            /**
             * Constructs a new PlacementFailed.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPlacementFailed);

            /** PlacementFailed byTeam. */
            public byTeam: proto.Team;

            /** PlacementFailed remainingDistance. */
            public remainingDistance: number;

            /**
             * Creates a new PlacementFailed instance using the specified properties.
             * @param [properties] Properties to set
             * @returns PlacementFailed instance
             */
            public static create(properties?: proto.GameEvent.IPlacementFailed): proto.GameEvent.PlacementFailed;

            /**
             * Encodes the specified PlacementFailed message. Does not implicitly {@link proto.GameEvent.PlacementFailed.verify|verify} messages.
             * @param message PlacementFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPlacementFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified PlacementFailed message, length delimited. Does not implicitly {@link proto.GameEvent.PlacementFailed.verify|verify} messages.
             * @param message PlacementFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPlacementFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a PlacementFailed message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns PlacementFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.PlacementFailed;

            /**
             * Decodes a PlacementFailed message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns PlacementFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.PlacementFailed;

            /**
             * Verifies a PlacementFailed message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a PlacementFailed message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns PlacementFailed
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.PlacementFailed;

            /**
             * Creates a plain object from a PlacementFailed message. Also converts values to other types if specified.
             * @param message PlacementFailed
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.PlacementFailed, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this PlacementFailed to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for PlacementFailed
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an UnsportingBehaviorMinor. */
        interface IUnsportingBehaviorMinor {

            /** UnsportingBehaviorMinor byTeam */
            byTeam: proto.Team;

            /** UnsportingBehaviorMinor reason */
            reason: string;
        }

        /** Represents an UnsportingBehaviorMinor. */
        class UnsportingBehaviorMinor implements IUnsportingBehaviorMinor {

            /**
             * Constructs a new UnsportingBehaviorMinor.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IUnsportingBehaviorMinor);

            /** UnsportingBehaviorMinor byTeam. */
            public byTeam: proto.Team;

            /** UnsportingBehaviorMinor reason. */
            public reason: string;

            /**
             * Creates a new UnsportingBehaviorMinor instance using the specified properties.
             * @param [properties] Properties to set
             * @returns UnsportingBehaviorMinor instance
             */
            public static create(properties?: proto.GameEvent.IUnsportingBehaviorMinor): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Encodes the specified UnsportingBehaviorMinor message. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMinor.verify|verify} messages.
             * @param message UnsportingBehaviorMinor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IUnsportingBehaviorMinor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified UnsportingBehaviorMinor message, length delimited. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMinor.verify|verify} messages.
             * @param message UnsportingBehaviorMinor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IUnsportingBehaviorMinor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an UnsportingBehaviorMinor message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns UnsportingBehaviorMinor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Decodes an UnsportingBehaviorMinor message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns UnsportingBehaviorMinor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Verifies an UnsportingBehaviorMinor message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an UnsportingBehaviorMinor message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns UnsportingBehaviorMinor
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Creates a plain object from an UnsportingBehaviorMinor message. Also converts values to other types if specified.
             * @param message UnsportingBehaviorMinor
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.UnsportingBehaviorMinor, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this UnsportingBehaviorMinor to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for UnsportingBehaviorMinor
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an UnsportingBehaviorMajor. */
        interface IUnsportingBehaviorMajor {

            /** UnsportingBehaviorMajor byTeam */
            byTeam: proto.Team;

            /** UnsportingBehaviorMajor reason */
            reason: string;
        }

        /** Represents an UnsportingBehaviorMajor. */
        class UnsportingBehaviorMajor implements IUnsportingBehaviorMajor {

            /**
             * Constructs a new UnsportingBehaviorMajor.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IUnsportingBehaviorMajor);

            /** UnsportingBehaviorMajor byTeam. */
            public byTeam: proto.Team;

            /** UnsportingBehaviorMajor reason. */
            public reason: string;

            /**
             * Creates a new UnsportingBehaviorMajor instance using the specified properties.
             * @param [properties] Properties to set
             * @returns UnsportingBehaviorMajor instance
             */
            public static create(properties?: proto.GameEvent.IUnsportingBehaviorMajor): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Encodes the specified UnsportingBehaviorMajor message. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMajor.verify|verify} messages.
             * @param message UnsportingBehaviorMajor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IUnsportingBehaviorMajor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified UnsportingBehaviorMajor message, length delimited. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMajor.verify|verify} messages.
             * @param message UnsportingBehaviorMajor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IUnsportingBehaviorMajor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an UnsportingBehaviorMajor message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns UnsportingBehaviorMajor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Decodes an UnsportingBehaviorMajor message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns UnsportingBehaviorMajor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Verifies an UnsportingBehaviorMajor message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an UnsportingBehaviorMajor message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns UnsportingBehaviorMajor
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Creates a plain object from an UnsportingBehaviorMajor message. Also converts values to other types if specified.
             * @param message UnsportingBehaviorMajor
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.UnsportingBehaviorMajor, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this UnsportingBehaviorMajor to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for UnsportingBehaviorMajor
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a KeeperHeldBall. */
        interface IKeeperHeldBall {

            /** KeeperHeldBall byTeam */
            byTeam: proto.Team;

            /** KeeperHeldBall location */
            location?: (proto.IVector2|null);

            /** KeeperHeldBall duration */
            duration?: (number|null);
        }

        /** Represents a KeeperHeldBall. */
        class KeeperHeldBall implements IKeeperHeldBall {

            /**
             * Constructs a new KeeperHeldBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IKeeperHeldBall);

            /** KeeperHeldBall byTeam. */
            public byTeam: proto.Team;

            /** KeeperHeldBall location. */
            public location?: (proto.IVector2|null);

            /** KeeperHeldBall duration. */
            public duration: number;

            /**
             * Creates a new KeeperHeldBall instance using the specified properties.
             * @param [properties] Properties to set
             * @returns KeeperHeldBall instance
             */
            public static create(properties?: proto.GameEvent.IKeeperHeldBall): proto.GameEvent.KeeperHeldBall;

            /**
             * Encodes the specified KeeperHeldBall message. Does not implicitly {@link proto.GameEvent.KeeperHeldBall.verify|verify} messages.
             * @param message KeeperHeldBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IKeeperHeldBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified KeeperHeldBall message, length delimited. Does not implicitly {@link proto.GameEvent.KeeperHeldBall.verify|verify} messages.
             * @param message KeeperHeldBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IKeeperHeldBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KeeperHeldBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KeeperHeldBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.KeeperHeldBall;

            /**
             * Decodes a KeeperHeldBall message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns KeeperHeldBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.KeeperHeldBall;

            /**
             * Verifies a KeeperHeldBall message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a KeeperHeldBall message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns KeeperHeldBall
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.KeeperHeldBall;

            /**
             * Creates a plain object from a KeeperHeldBall message. Also converts values to other types if specified.
             * @param message KeeperHeldBall
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.KeeperHeldBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this KeeperHeldBall to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for KeeperHeldBall
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a PlacementSucceeded. */
        interface IPlacementSucceeded {

            /** PlacementSucceeded byTeam */
            byTeam: proto.Team;

            /** PlacementSucceeded timeTaken */
            timeTaken?: (number|null);

            /** PlacementSucceeded precision */
            precision?: (number|null);

            /** PlacementSucceeded distance */
            distance?: (number|null);
        }

        /** Represents a PlacementSucceeded. */
        class PlacementSucceeded implements IPlacementSucceeded {

            /**
             * Constructs a new PlacementSucceeded.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPlacementSucceeded);

            /** PlacementSucceeded byTeam. */
            public byTeam: proto.Team;

            /** PlacementSucceeded timeTaken. */
            public timeTaken: number;

            /** PlacementSucceeded precision. */
            public precision: number;

            /** PlacementSucceeded distance. */
            public distance: number;

            /**
             * Creates a new PlacementSucceeded instance using the specified properties.
             * @param [properties] Properties to set
             * @returns PlacementSucceeded instance
             */
            public static create(properties?: proto.GameEvent.IPlacementSucceeded): proto.GameEvent.PlacementSucceeded;

            /**
             * Encodes the specified PlacementSucceeded message. Does not implicitly {@link proto.GameEvent.PlacementSucceeded.verify|verify} messages.
             * @param message PlacementSucceeded message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPlacementSucceeded, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified PlacementSucceeded message, length delimited. Does not implicitly {@link proto.GameEvent.PlacementSucceeded.verify|verify} messages.
             * @param message PlacementSucceeded message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPlacementSucceeded, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a PlacementSucceeded message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns PlacementSucceeded
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.PlacementSucceeded;

            /**
             * Decodes a PlacementSucceeded message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns PlacementSucceeded
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.PlacementSucceeded;

            /**
             * Verifies a PlacementSucceeded message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a PlacementSucceeded message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns PlacementSucceeded
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.PlacementSucceeded;

            /**
             * Creates a plain object from a PlacementSucceeded message. Also converts values to other types if specified.
             * @param message PlacementSucceeded
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.PlacementSucceeded, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this PlacementSucceeded to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for PlacementSucceeded
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Prepared. */
        interface IPrepared {

            /** Prepared timeTaken */
            timeTaken?: (number|null);
        }

        /** Represents a Prepared. */
        class Prepared implements IPrepared {

            /**
             * Constructs a new Prepared.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPrepared);

            /** Prepared timeTaken. */
            public timeTaken: number;

            /**
             * Creates a new Prepared instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Prepared instance
             */
            public static create(properties?: proto.GameEvent.IPrepared): proto.GameEvent.Prepared;

            /**
             * Encodes the specified Prepared message. Does not implicitly {@link proto.GameEvent.Prepared.verify|verify} messages.
             * @param message Prepared message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPrepared, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Prepared message, length delimited. Does not implicitly {@link proto.GameEvent.Prepared.verify|verify} messages.
             * @param message Prepared message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPrepared, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Prepared message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Prepared
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.Prepared;

            /**
             * Decodes a Prepared message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Prepared
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.Prepared;

            /**
             * Verifies a Prepared message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Prepared message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Prepared
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.Prepared;

            /**
             * Creates a plain object from a Prepared message. Also converts values to other types if specified.
             * @param message Prepared
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.Prepared, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Prepared to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Prepared
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotSubstitution. */
        interface IBotSubstitution {

            /** BotSubstitution byTeam */
            byTeam: proto.Team;
        }

        /** Represents a BotSubstitution. */
        class BotSubstitution implements IBotSubstitution {

            /**
             * Constructs a new BotSubstitution.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotSubstitution);

            /** BotSubstitution byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new BotSubstitution instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotSubstitution instance
             */
            public static create(properties?: proto.GameEvent.IBotSubstitution): proto.GameEvent.BotSubstitution;

            /**
             * Encodes the specified BotSubstitution message. Does not implicitly {@link proto.GameEvent.BotSubstitution.verify|verify} messages.
             * @param message BotSubstitution message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotSubstitution, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotSubstitution message, length delimited. Does not implicitly {@link proto.GameEvent.BotSubstitution.verify|verify} messages.
             * @param message BotSubstitution message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotSubstitution, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotSubstitution message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotSubstitution
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotSubstitution;

            /**
             * Decodes a BotSubstitution message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotSubstitution
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotSubstitution;

            /**
             * Verifies a BotSubstitution message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotSubstitution message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotSubstitution
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotSubstitution;

            /**
             * Creates a plain object from a BotSubstitution message. Also converts values to other types if specified.
             * @param message BotSubstitution
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotSubstitution, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotSubstitution to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotSubstitution
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a ChallengeFlag. */
        interface IChallengeFlag {

            /** ChallengeFlag byTeam */
            byTeam: proto.Team;
        }

        /** Represents a ChallengeFlag. */
        class ChallengeFlag implements IChallengeFlag {

            /**
             * Constructs a new ChallengeFlag.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IChallengeFlag);

            /** ChallengeFlag byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new ChallengeFlag instance using the specified properties.
             * @param [properties] Properties to set
             * @returns ChallengeFlag instance
             */
            public static create(properties?: proto.GameEvent.IChallengeFlag): proto.GameEvent.ChallengeFlag;

            /**
             * Encodes the specified ChallengeFlag message. Does not implicitly {@link proto.GameEvent.ChallengeFlag.verify|verify} messages.
             * @param message ChallengeFlag message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IChallengeFlag, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified ChallengeFlag message, length delimited. Does not implicitly {@link proto.GameEvent.ChallengeFlag.verify|verify} messages.
             * @param message ChallengeFlag message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IChallengeFlag, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ChallengeFlag message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ChallengeFlag
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.ChallengeFlag;

            /**
             * Decodes a ChallengeFlag message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns ChallengeFlag
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.ChallengeFlag;

            /**
             * Verifies a ChallengeFlag message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a ChallengeFlag message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns ChallengeFlag
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.ChallengeFlag;

            /**
             * Creates a plain object from a ChallengeFlag message. Also converts values to other types if specified.
             * @param message ChallengeFlag
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.ChallengeFlag, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this ChallengeFlag to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for ChallengeFlag
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an EmergencyStop. */
        interface IEmergencyStop {

            /** EmergencyStop byTeam */
            byTeam: proto.Team;
        }

        /** Represents an EmergencyStop. */
        class EmergencyStop implements IEmergencyStop {

            /**
             * Constructs a new EmergencyStop.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IEmergencyStop);

            /** EmergencyStop byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new EmergencyStop instance using the specified properties.
             * @param [properties] Properties to set
             * @returns EmergencyStop instance
             */
            public static create(properties?: proto.GameEvent.IEmergencyStop): proto.GameEvent.EmergencyStop;

            /**
             * Encodes the specified EmergencyStop message. Does not implicitly {@link proto.GameEvent.EmergencyStop.verify|verify} messages.
             * @param message EmergencyStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IEmergencyStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified EmergencyStop message, length delimited. Does not implicitly {@link proto.GameEvent.EmergencyStop.verify|verify} messages.
             * @param message EmergencyStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IEmergencyStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an EmergencyStop message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns EmergencyStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.EmergencyStop;

            /**
             * Decodes an EmergencyStop message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns EmergencyStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.EmergencyStop;

            /**
             * Verifies an EmergencyStop message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an EmergencyStop message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns EmergencyStop
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.EmergencyStop;

            /**
             * Creates a plain object from an EmergencyStop message. Also converts values to other types if specified.
             * @param message EmergencyStop
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.EmergencyStop, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this EmergencyStop to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for EmergencyStop
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a TooManyRobots. */
        interface ITooManyRobots {

            /** TooManyRobots byTeam */
            byTeam: proto.Team;

            /** TooManyRobots numRobotsAllowed */
            numRobotsAllowed?: (number|null);

            /** TooManyRobots numRobotsOnField */
            numRobotsOnField?: (number|null);

            /** TooManyRobots ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents a TooManyRobots. */
        class TooManyRobots implements ITooManyRobots {

            /**
             * Constructs a new TooManyRobots.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.ITooManyRobots);

            /** TooManyRobots byTeam. */
            public byTeam: proto.Team;

            /** TooManyRobots numRobotsAllowed. */
            public numRobotsAllowed: number;

            /** TooManyRobots numRobotsOnField. */
            public numRobotsOnField: number;

            /** TooManyRobots ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new TooManyRobots instance using the specified properties.
             * @param [properties] Properties to set
             * @returns TooManyRobots instance
             */
            public static create(properties?: proto.GameEvent.ITooManyRobots): proto.GameEvent.TooManyRobots;

            /**
             * Encodes the specified TooManyRobots message. Does not implicitly {@link proto.GameEvent.TooManyRobots.verify|verify} messages.
             * @param message TooManyRobots message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.ITooManyRobots, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified TooManyRobots message, length delimited. Does not implicitly {@link proto.GameEvent.TooManyRobots.verify|verify} messages.
             * @param message TooManyRobots message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.ITooManyRobots, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a TooManyRobots message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns TooManyRobots
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.TooManyRobots;

            /**
             * Decodes a TooManyRobots message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns TooManyRobots
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.TooManyRobots;

            /**
             * Verifies a TooManyRobots message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a TooManyRobots message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns TooManyRobots
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.TooManyRobots;

            /**
             * Creates a plain object from a TooManyRobots message. Also converts values to other types if specified.
             * @param message TooManyRobots
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.TooManyRobots, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this TooManyRobots to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for TooManyRobots
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BoundaryCrossing. */
        interface IBoundaryCrossing {

            /** BoundaryCrossing byTeam */
            byTeam: proto.Team;

            /** BoundaryCrossing location */
            location?: (proto.IVector2|null);
        }

        /** Represents a BoundaryCrossing. */
        class BoundaryCrossing implements IBoundaryCrossing {

            /**
             * Constructs a new BoundaryCrossing.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBoundaryCrossing);

            /** BoundaryCrossing byTeam. */
            public byTeam: proto.Team;

            /** BoundaryCrossing location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new BoundaryCrossing instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BoundaryCrossing instance
             */
            public static create(properties?: proto.GameEvent.IBoundaryCrossing): proto.GameEvent.BoundaryCrossing;

            /**
             * Encodes the specified BoundaryCrossing message. Does not implicitly {@link proto.GameEvent.BoundaryCrossing.verify|verify} messages.
             * @param message BoundaryCrossing message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBoundaryCrossing, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BoundaryCrossing message, length delimited. Does not implicitly {@link proto.GameEvent.BoundaryCrossing.verify|verify} messages.
             * @param message BoundaryCrossing message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBoundaryCrossing, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BoundaryCrossing message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BoundaryCrossing
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BoundaryCrossing;

            /**
             * Decodes a BoundaryCrossing message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BoundaryCrossing
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BoundaryCrossing;

            /**
             * Verifies a BoundaryCrossing message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BoundaryCrossing message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BoundaryCrossing
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BoundaryCrossing;

            /**
             * Creates a plain object from a BoundaryCrossing message. Also converts values to other types if specified.
             * @param message BoundaryCrossing
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BoundaryCrossing, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BoundaryCrossing to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BoundaryCrossing
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a PenaltyKickFailed. */
        interface IPenaltyKickFailed {

            /** PenaltyKickFailed byTeam */
            byTeam: proto.Team;

            /** PenaltyKickFailed location */
            location?: (proto.IVector2|null);
        }

        /** Represents a PenaltyKickFailed. */
        class PenaltyKickFailed implements IPenaltyKickFailed {

            /**
             * Constructs a new PenaltyKickFailed.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPenaltyKickFailed);

            /** PenaltyKickFailed byTeam. */
            public byTeam: proto.Team;

            /** PenaltyKickFailed location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new PenaltyKickFailed instance using the specified properties.
             * @param [properties] Properties to set
             * @returns PenaltyKickFailed instance
             */
            public static create(properties?: proto.GameEvent.IPenaltyKickFailed): proto.GameEvent.PenaltyKickFailed;

            /**
             * Encodes the specified PenaltyKickFailed message. Does not implicitly {@link proto.GameEvent.PenaltyKickFailed.verify|verify} messages.
             * @param message PenaltyKickFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPenaltyKickFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified PenaltyKickFailed message, length delimited. Does not implicitly {@link proto.GameEvent.PenaltyKickFailed.verify|verify} messages.
             * @param message PenaltyKickFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPenaltyKickFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a PenaltyKickFailed message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns PenaltyKickFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.PenaltyKickFailed;

            /**
             * Decodes a PenaltyKickFailed message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns PenaltyKickFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.PenaltyKickFailed;

            /**
             * Verifies a PenaltyKickFailed message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a PenaltyKickFailed message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns PenaltyKickFailed
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.PenaltyKickFailed;

            /**
             * Creates a plain object from a PenaltyKickFailed message. Also converts values to other types if specified.
             * @param message PenaltyKickFailed
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.PenaltyKickFailed, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this PenaltyKickFailed to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for PenaltyKickFailed
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Type enum. */
        enum Type {
            UNKNOWN_GAME_EVENT_TYPE = 0,
            BALL_LEFT_FIELD_TOUCH_LINE = 6,
            BALL_LEFT_FIELD_GOAL_LINE = 7,
            AIMLESS_KICK = 11,
            ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA = 19,
            DEFENDER_IN_DEFENSE_AREA = 31,
            BOUNDARY_CROSSING = 41,
            KEEPER_HELD_BALL = 13,
            BOT_DRIBBLED_BALL_TOO_FAR = 17,
            BOT_PUSHED_BOT = 24,
            BOT_HELD_BALL_DELIBERATELY = 26,
            BOT_TIPPED_OVER = 27,
            ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA = 15,
            BOT_KICKED_BALL_TOO_FAST = 18,
            BOT_CRASH_UNIQUE = 22,
            BOT_CRASH_DRAWN = 21,
            DEFENDER_TOO_CLOSE_TO_KICK_POINT = 29,
            BOT_TOO_FAST_IN_STOP = 28,
            BOT_INTERFERED_PLACEMENT = 20,
            POSSIBLE_GOAL = 39,
            GOAL = 8,
            INVALID_GOAL = 42,
            ATTACKER_DOUBLE_TOUCHED_BALL = 14,
            PLACEMENT_SUCCEEDED = 5,
            PENALTY_KICK_FAILED = 43,
            NO_PROGRESS_IN_GAME = 2,
            PLACEMENT_FAILED = 3,
            MULTIPLE_CARDS = 32,
            MULTIPLE_FOULS = 34,
            BOT_SUBSTITUTION = 37,
            TOO_MANY_ROBOTS = 38,
            CHALLENGE_FLAG = 44,
            EMERGENCY_STOP = 45,
            UNSPORTING_BEHAVIOR_MINOR = 35,
            UNSPORTING_BEHAVIOR_MAJOR = 36
        }
    }

    /** Team enum. */
    enum Team {
        UNKNOWN = 0,
        YELLOW = 1,
        BLUE = 2
    }

    /** Properties of a RobotId. */
    interface IRobotId {

        /** RobotId id */
        id?: (number|null);

        /** RobotId team */
        team?: (proto.Team|null);
    }

    /** Represents a RobotId. */
    class RobotId implements IRobotId {

        /**
         * Constructs a new RobotId.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotId);

        /** RobotId id. */
        public id: number;

        /** RobotId team. */
        public team: proto.Team;

        /**
         * Creates a new RobotId instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotId instance
         */
        public static create(properties?: proto.IRobotId): proto.RobotId;

        /**
         * Encodes the specified RobotId message. Does not implicitly {@link proto.RobotId.verify|verify} messages.
         * @param message RobotId message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotId, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotId message, length delimited. Does not implicitly {@link proto.RobotId.verify|verify} messages.
         * @param message RobotId message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotId, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotId message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotId
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotId;

        /**
         * Decodes a RobotId message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotId
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotId;

        /**
         * Verifies a RobotId message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotId message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotId
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotId;

        /**
         * Creates a plain object from a RobotId message. Also converts values to other types if specified.
         * @param message RobotId
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotId, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotId to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotId
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Division enum. */
    enum Division {
        DIV_UNKNOWN = 0,
        DIV_A = 1,
        DIV_B = 2
    }

    /** Properties of a Vector2. */
    interface IVector2 {

        /** Vector2 x */
        x: number;

        /** Vector2 y */
        y: number;
    }

    /** Represents a Vector2. */
    class Vector2 implements IVector2 {

        /**
         * Constructs a new Vector2.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IVector2);

        /** Vector2 x. */
        public x: number;

        /** Vector2 y. */
        public y: number;

        /**
         * Creates a new Vector2 instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Vector2 instance
         */
        public static create(properties?: proto.IVector2): proto.Vector2;

        /**
         * Encodes the specified Vector2 message. Does not implicitly {@link proto.Vector2.verify|verify} messages.
         * @param message Vector2 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IVector2, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Vector2 message, length delimited. Does not implicitly {@link proto.Vector2.verify|verify} messages.
         * @param message Vector2 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IVector2, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Vector2 message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Vector2
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Vector2;

        /**
         * Decodes a Vector2 message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Vector2
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Vector2;

        /**
         * Verifies a Vector2 message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Vector2 message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Vector2
         */
        public static fromObject(object: { [k: string]: any }): proto.Vector2;

        /**
         * Creates a plain object from a Vector2 message. Also converts values to other types if specified.
         * @param message Vector2
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Vector2, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Vector2 to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Vector2
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Vector3. */
    interface IVector3 {

        /** Vector3 x */
        x: number;

        /** Vector3 y */
        y: number;

        /** Vector3 z */
        z: number;
    }

    /** Represents a Vector3. */
    class Vector3 implements IVector3 {

        /**
         * Constructs a new Vector3.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IVector3);

        /** Vector3 x. */
        public x: number;

        /** Vector3 y. */
        public y: number;

        /** Vector3 z. */
        public z: number;

        /**
         * Creates a new Vector3 instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Vector3 instance
         */
        public static create(properties?: proto.IVector3): proto.Vector3;

        /**
         * Encodes the specified Vector3 message. Does not implicitly {@link proto.Vector3.verify|verify} messages.
         * @param message Vector3 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IVector3, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Vector3 message, length delimited. Does not implicitly {@link proto.Vector3.verify|verify} messages.
         * @param message Vector3 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IVector3, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Vector3 message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Vector3
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Vector3;

        /**
         * Decodes a Vector3 message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Vector3
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Vector3;

        /**
         * Verifies a Vector3 message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Vector3 message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Vector3
         */
        public static fromObject(object: { [k: string]: any }): proto.Vector3;

        /**
         * Creates a plain object from a Vector3 message. Also converts values to other types if specified.
         * @param message Vector3
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Vector3, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Vector3 to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Vector3
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotCommand. */
    interface IRobotCommand {

        /** RobotCommand id */
        id?: (number|null);

        /** RobotCommand velocityX */
        velocityX?: (number|null);

        /** RobotCommand velocityY */
        velocityY?: (number|null);

        /** RobotCommand angle */
        angle?: (number|null);

        /** RobotCommand angularVelocity */
        angularVelocity?: (number|null);

        /** RobotCommand useAngularVelocity */
        useAngularVelocity?: (boolean|null);

        /** RobotCommand cameraAngleOfRobot */
        cameraAngleOfRobot?: (number|null);

        /** RobotCommand cameraAngleOfRobotIsSet */
        cameraAngleOfRobotIsSet?: (boolean|null);

        /** RobotCommand kickSpeed */
        kickSpeed?: (number|null);

        /** RobotCommand waitForBall */
        waitForBall?: (boolean|null);

        /** RobotCommand kickAtAngle */
        kickAtAngle?: (boolean|null);

        /** RobotCommand kickType */
        kickType?: (proto.RobotCommand.KickType|null);

        /** RobotCommand dribblerSpeed */
        dribblerSpeed?: (number|null);

        /** RobotCommand ignorePacket */
        ignorePacket?: (boolean|null);
    }

    /** Represents a RobotCommand. */
    class RobotCommand implements IRobotCommand {

        /**
         * Constructs a new RobotCommand.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotCommand);

        /** RobotCommand id. */
        public id: number;

        /** RobotCommand velocityX. */
        public velocityX: number;

        /** RobotCommand velocityY. */
        public velocityY: number;

        /** RobotCommand angle. */
        public angle: number;

        /** RobotCommand angularVelocity. */
        public angularVelocity: number;

        /** RobotCommand useAngularVelocity. */
        public useAngularVelocity: boolean;

        /** RobotCommand cameraAngleOfRobot. */
        public cameraAngleOfRobot: number;

        /** RobotCommand cameraAngleOfRobotIsSet. */
        public cameraAngleOfRobotIsSet: boolean;

        /** RobotCommand kickSpeed. */
        public kickSpeed: number;

        /** RobotCommand waitForBall. */
        public waitForBall: boolean;

        /** RobotCommand kickAtAngle. */
        public kickAtAngle: boolean;

        /** RobotCommand kickType. */
        public kickType: proto.RobotCommand.KickType;

        /** RobotCommand dribblerSpeed. */
        public dribblerSpeed: number;

        /** RobotCommand ignorePacket. */
        public ignorePacket: boolean;

        /**
         * Creates a new RobotCommand instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotCommand instance
         */
        public static create(properties?: proto.IRobotCommand): proto.RobotCommand;

        /**
         * Encodes the specified RobotCommand message. Does not implicitly {@link proto.RobotCommand.verify|verify} messages.
         * @param message RobotCommand message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotCommand, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotCommand message, length delimited. Does not implicitly {@link proto.RobotCommand.verify|verify} messages.
         * @param message RobotCommand message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotCommand, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotCommand message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotCommand
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotCommand;

        /**
         * Decodes a RobotCommand message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotCommand
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotCommand;

        /**
         * Verifies a RobotCommand message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotCommand message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotCommand
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotCommand;

        /**
         * Creates a plain object from a RobotCommand message. Also converts values to other types if specified.
         * @param message RobotCommand
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotCommand, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotCommand to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotCommand
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace RobotCommand {

        /** KickType enum. */
        enum KickType {
            NO_KICK = 0,
            KICK = 1,
            CHIP = 2
        }
    }

    /** Properties of a RobotCommands. */
    interface IRobotCommands {

        /** RobotCommands robotCommands */
        robotCommands?: (proto.IRobotCommand[]|null);
    }

    /** Represents a RobotCommands. */
    class RobotCommands implements IRobotCommands {

        /**
         * Constructs a new RobotCommands.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotCommands);

        /** RobotCommands robotCommands. */
        public robotCommands: proto.IRobotCommand[];

        /**
         * Creates a new RobotCommands instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotCommands instance
         */
        public static create(properties?: proto.IRobotCommands): proto.RobotCommands;

        /**
         * Encodes the specified RobotCommands message. Does not implicitly {@link proto.RobotCommands.verify|verify} messages.
         * @param message RobotCommands message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotCommands, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotCommands message, length delimited. Does not implicitly {@link proto.RobotCommands.verify|verify} messages.
         * @param message RobotCommands message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotCommands, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotCommands message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotCommands
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotCommands;

        /**
         * Decodes a RobotCommands message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotCommands
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotCommands;

        /**
         * Verifies a RobotCommands message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotCommands message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotCommands
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotCommands;

        /**
         * Creates a plain object from a RobotCommands message. Also converts values to other types if specified.
         * @param message RobotCommands
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotCommands, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotCommands to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotCommands
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** RobotTeam enum. */
    enum RobotTeam {
        YELLOW_TEAM = 0,
        BLUE_TEAM = 1
    }

    /** RobotFeedbackSource enum. */
    enum RobotFeedbackSource {
        SIMULATOR = 0,
        BASESTATION = 1
    }

    /** Properties of a RobotFeedback. */
    interface IRobotFeedback {

        /** RobotFeedback id */
        id?: (number|null);

        /** RobotFeedback ballSensorSeesBall */
        ballSensorSeesBall?: (boolean|null);

        /** RobotFeedback ballPosition */
        ballPosition?: (number|null);

        /** RobotFeedback ballSensorIsWorking */
        ballSensorIsWorking?: (boolean|null);

        /** RobotFeedback dribblerSeesBall */
        dribblerSeesBall?: (boolean|null);

        /** RobotFeedback estimatedVelocityX */
        estimatedVelocityX?: (number|null);

        /** RobotFeedback estimatedVelocityY */
        estimatedVelocityY?: (number|null);

        /** RobotFeedback estimatedAngle */
        estimatedAngle?: (number|null);

        /** RobotFeedback xsensIsCalibrated */
        xsensIsCalibrated?: (boolean|null);

        /** RobotFeedback capacitorIsCharged */
        capacitorIsCharged?: (boolean|null);

        /** RobotFeedback wheelsLocked */
        wheelsLocked?: (number|null);

        /** RobotFeedback wheelsBraking */
        wheelsBraking?: (number|null);

        /** RobotFeedback batteryLevel */
        batteryLevel?: (number|null);

        /** RobotFeedback signalStrength */
        signalStrength?: (number|null);
    }

    /** Represents a RobotFeedback. */
    class RobotFeedback implements IRobotFeedback {

        /**
         * Constructs a new RobotFeedback.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotFeedback);

        /** RobotFeedback id. */
        public id: number;

        /** RobotFeedback ballSensorSeesBall. */
        public ballSensorSeesBall: boolean;

        /** RobotFeedback ballPosition. */
        public ballPosition: number;

        /** RobotFeedback ballSensorIsWorking. */
        public ballSensorIsWorking: boolean;

        /** RobotFeedback dribblerSeesBall. */
        public dribblerSeesBall: boolean;

        /** RobotFeedback estimatedVelocityX. */
        public estimatedVelocityX: number;

        /** RobotFeedback estimatedVelocityY. */
        public estimatedVelocityY: number;

        /** RobotFeedback estimatedAngle. */
        public estimatedAngle: number;

        /** RobotFeedback xsensIsCalibrated. */
        public xsensIsCalibrated: boolean;

        /** RobotFeedback capacitorIsCharged. */
        public capacitorIsCharged: boolean;

        /** RobotFeedback wheelsLocked. */
        public wheelsLocked: number;

        /** RobotFeedback wheelsBraking. */
        public wheelsBraking: number;

        /** RobotFeedback batteryLevel. */
        public batteryLevel: number;

        /** RobotFeedback signalStrength. */
        public signalStrength: number;

        /**
         * Creates a new RobotFeedback instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotFeedback instance
         */
        public static create(properties?: proto.IRobotFeedback): proto.RobotFeedback;

        /**
         * Encodes the specified RobotFeedback message. Does not implicitly {@link proto.RobotFeedback.verify|verify} messages.
         * @param message RobotFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotFeedback message, length delimited. Does not implicitly {@link proto.RobotFeedback.verify|verify} messages.
         * @param message RobotFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotFeedback message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotFeedback;

        /**
         * Decodes a RobotFeedback message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotFeedback;

        /**
         * Verifies a RobotFeedback message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotFeedback message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotFeedback
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotFeedback;

        /**
         * Creates a plain object from a RobotFeedback message. Also converts values to other types if specified.
         * @param message RobotFeedback
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotFeedback, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotFeedback to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotFeedback
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotsFeedback. */
    interface IRobotsFeedback {

        /** RobotsFeedback team */
        team?: (proto.RobotTeam|null);

        /** RobotsFeedback source */
        source?: (proto.RobotFeedbackSource|null);

        /** RobotsFeedback robotsFeedback */
        robotsFeedback?: (proto.IRobotFeedback[]|null);
    }

    /** Represents a RobotsFeedback. */
    class RobotsFeedback implements IRobotsFeedback {

        /**
         * Constructs a new RobotsFeedback.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotsFeedback);

        /** RobotsFeedback team. */
        public team: proto.RobotTeam;

        /** RobotsFeedback source. */
        public source: proto.RobotFeedbackSource;

        /** RobotsFeedback robotsFeedback. */
        public robotsFeedback: proto.IRobotFeedback[];

        /**
         * Creates a new RobotsFeedback instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotsFeedback instance
         */
        public static create(properties?: proto.IRobotsFeedback): proto.RobotsFeedback;

        /**
         * Encodes the specified RobotsFeedback message. Does not implicitly {@link proto.RobotsFeedback.verify|verify} messages.
         * @param message RobotsFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotsFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotsFeedback message, length delimited. Does not implicitly {@link proto.RobotsFeedback.verify|verify} messages.
         * @param message RobotsFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotsFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotsFeedback message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotsFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotsFeedback;

        /**
         * Decodes a RobotsFeedback message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotsFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotsFeedback;

        /**
         * Verifies a RobotsFeedback message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotsFeedback message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotsFeedback
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotsFeedback;

        /**
         * Creates a plain object from a RobotsFeedback message. Also converts values to other types if specified.
         * @param message RobotsFeedback
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotsFeedback, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotsFeedback to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotsFeedback
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SimulationBallLocation. */
    interface ISimulationBallLocation {

        /** SimulationBallLocation x */
        x?: (number|null);

        /** SimulationBallLocation y */
        y?: (number|null);

        /** SimulationBallLocation z */
        z?: (number|null);

        /** SimulationBallLocation xVelocity */
        xVelocity?: (number|null);

        /** SimulationBallLocation yVelocity */
        yVelocity?: (number|null);

        /** SimulationBallLocation zVelocity */
        zVelocity?: (number|null);

        /** SimulationBallLocation velocityInRolling */
        velocityInRolling?: (boolean|null);

        /** SimulationBallLocation teleportSafely */
        teleportSafely?: (boolean|null);

        /** SimulationBallLocation byForce */
        byForce?: (boolean|null);
    }

    /** Represents a SimulationBallLocation. */
    class SimulationBallLocation implements ISimulationBallLocation {

        /**
         * Constructs a new SimulationBallLocation.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISimulationBallLocation);

        /** SimulationBallLocation x. */
        public x: number;

        /** SimulationBallLocation y. */
        public y: number;

        /** SimulationBallLocation z. */
        public z: number;

        /** SimulationBallLocation xVelocity. */
        public xVelocity: number;

        /** SimulationBallLocation yVelocity. */
        public yVelocity: number;

        /** SimulationBallLocation zVelocity. */
        public zVelocity: number;

        /** SimulationBallLocation velocityInRolling. */
        public velocityInRolling: boolean;

        /** SimulationBallLocation teleportSafely. */
        public teleportSafely: boolean;

        /** SimulationBallLocation byForce. */
        public byForce: boolean;

        /**
         * Creates a new SimulationBallLocation instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SimulationBallLocation instance
         */
        public static create(properties?: proto.ISimulationBallLocation): proto.SimulationBallLocation;

        /**
         * Encodes the specified SimulationBallLocation message. Does not implicitly {@link proto.SimulationBallLocation.verify|verify} messages.
         * @param message SimulationBallLocation message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISimulationBallLocation, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SimulationBallLocation message, length delimited. Does not implicitly {@link proto.SimulationBallLocation.verify|verify} messages.
         * @param message SimulationBallLocation message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISimulationBallLocation, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SimulationBallLocation message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SimulationBallLocation
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SimulationBallLocation;

        /**
         * Decodes a SimulationBallLocation message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SimulationBallLocation
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SimulationBallLocation;

        /**
         * Verifies a SimulationBallLocation message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SimulationBallLocation message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SimulationBallLocation
         */
        public static fromObject(object: { [k: string]: any }): proto.SimulationBallLocation;

        /**
         * Creates a plain object from a SimulationBallLocation message. Also converts values to other types if specified.
         * @param message SimulationBallLocation
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SimulationBallLocation, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SimulationBallLocation to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SimulationBallLocation
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SimulationRobotLocation. */
    interface ISimulationRobotLocation {

        /** SimulationRobotLocation id */
        id?: (number|null);

        /** SimulationRobotLocation isTeamYellow */
        isTeamYellow?: (boolean|null);

        /** SimulationRobotLocation x */
        x?: (number|null);

        /** SimulationRobotLocation y */
        y?: (number|null);

        /** SimulationRobotLocation xVelocity */
        xVelocity?: (number|null);

        /** SimulationRobotLocation yVelocity */
        yVelocity?: (number|null);

        /** SimulationRobotLocation angularVelocity */
        angularVelocity?: (number|null);

        /** SimulationRobotLocation orientation */
        orientation?: (number|null);

        /** SimulationRobotLocation presentOnField */
        presentOnField?: (boolean|null);

        /** SimulationRobotLocation byForce */
        byForce?: (boolean|null);
    }

    /** Represents a SimulationRobotLocation. */
    class SimulationRobotLocation implements ISimulationRobotLocation {

        /**
         * Constructs a new SimulationRobotLocation.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISimulationRobotLocation);

        /** SimulationRobotLocation id. */
        public id: number;

        /** SimulationRobotLocation isTeamYellow. */
        public isTeamYellow: boolean;

        /** SimulationRobotLocation x. */
        public x: number;

        /** SimulationRobotLocation y. */
        public y: number;

        /** SimulationRobotLocation xVelocity. */
        public xVelocity: number;

        /** SimulationRobotLocation yVelocity. */
        public yVelocity: number;

        /** SimulationRobotLocation angularVelocity. */
        public angularVelocity: number;

        /** SimulationRobotLocation orientation. */
        public orientation: number;

        /** SimulationRobotLocation presentOnField. */
        public presentOnField: boolean;

        /** SimulationRobotLocation byForce. */
        public byForce: boolean;

        /**
         * Creates a new SimulationRobotLocation instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SimulationRobotLocation instance
         */
        public static create(properties?: proto.ISimulationRobotLocation): proto.SimulationRobotLocation;

        /**
         * Encodes the specified SimulationRobotLocation message. Does not implicitly {@link proto.SimulationRobotLocation.verify|verify} messages.
         * @param message SimulationRobotLocation message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISimulationRobotLocation, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SimulationRobotLocation message, length delimited. Does not implicitly {@link proto.SimulationRobotLocation.verify|verify} messages.
         * @param message SimulationRobotLocation message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISimulationRobotLocation, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SimulationRobotLocation message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SimulationRobotLocation
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SimulationRobotLocation;

        /**
         * Decodes a SimulationRobotLocation message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SimulationRobotLocation
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SimulationRobotLocation;

        /**
         * Verifies a SimulationRobotLocation message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SimulationRobotLocation message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SimulationRobotLocation
         */
        public static fromObject(object: { [k: string]: any }): proto.SimulationRobotLocation;

        /**
         * Creates a plain object from a SimulationRobotLocation message. Also converts values to other types if specified.
         * @param message SimulationRobotLocation
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SimulationRobotLocation, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SimulationRobotLocation to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SimulationRobotLocation
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SimulationRobotProperties. */
    interface ISimulationRobotProperties {

        /** SimulationRobotProperties id */
        id?: (number|null);

        /** SimulationRobotProperties isTeamYellow */
        isTeamYellow?: (boolean|null);

        /** SimulationRobotProperties radius */
        radius?: (number|null);

        /** SimulationRobotProperties height */
        height?: (number|null);

        /** SimulationRobotProperties mass */
        mass?: (number|null);

        /** SimulationRobotProperties maxKickSpeed */
        maxKickSpeed?: (number|null);

        /** SimulationRobotProperties maxChipSpeed */
        maxChipSpeed?: (number|null);

        /** SimulationRobotProperties centerToDribblerDistance */
        centerToDribblerDistance?: (number|null);

        /** SimulationRobotProperties maxAcceleration */
        maxAcceleration?: (number|null);

        /** SimulationRobotProperties maxAngularAcceleration */
        maxAngularAcceleration?: (number|null);

        /** SimulationRobotProperties maxDeceleration */
        maxDeceleration?: (number|null);

        /** SimulationRobotProperties maxAngularDeceleration */
        maxAngularDeceleration?: (number|null);

        /** SimulationRobotProperties maxVelocity */
        maxVelocity?: (number|null);

        /** SimulationRobotProperties maxAngularVelocity */
        maxAngularVelocity?: (number|null);

        /** SimulationRobotProperties frontRightWheelAngle */
        frontRightWheelAngle?: (number|null);

        /** SimulationRobotProperties backRightWheelAngle */
        backRightWheelAngle?: (number|null);

        /** SimulationRobotProperties backLeftWheelAngle */
        backLeftWheelAngle?: (number|null);

        /** SimulationRobotProperties frontLeftWheelAngle */
        frontLeftWheelAngle?: (number|null);
    }

    /** Represents a SimulationRobotProperties. */
    class SimulationRobotProperties implements ISimulationRobotProperties {

        /**
         * Constructs a new SimulationRobotProperties.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISimulationRobotProperties);

        /** SimulationRobotProperties id. */
        public id: number;

        /** SimulationRobotProperties isTeamYellow. */
        public isTeamYellow: boolean;

        /** SimulationRobotProperties radius. */
        public radius: number;

        /** SimulationRobotProperties height. */
        public height: number;

        /** SimulationRobotProperties mass. */
        public mass: number;

        /** SimulationRobotProperties maxKickSpeed. */
        public maxKickSpeed: number;

        /** SimulationRobotProperties maxChipSpeed. */
        public maxChipSpeed: number;

        /** SimulationRobotProperties centerToDribblerDistance. */
        public centerToDribblerDistance: number;

        /** SimulationRobotProperties maxAcceleration. */
        public maxAcceleration: number;

        /** SimulationRobotProperties maxAngularAcceleration. */
        public maxAngularAcceleration: number;

        /** SimulationRobotProperties maxDeceleration. */
        public maxDeceleration: number;

        /** SimulationRobotProperties maxAngularDeceleration. */
        public maxAngularDeceleration: number;

        /** SimulationRobotProperties maxVelocity. */
        public maxVelocity: number;

        /** SimulationRobotProperties maxAngularVelocity. */
        public maxAngularVelocity: number;

        /** SimulationRobotProperties frontRightWheelAngle. */
        public frontRightWheelAngle: number;

        /** SimulationRobotProperties backRightWheelAngle. */
        public backRightWheelAngle: number;

        /** SimulationRobotProperties backLeftWheelAngle. */
        public backLeftWheelAngle: number;

        /** SimulationRobotProperties frontLeftWheelAngle. */
        public frontLeftWheelAngle: number;

        /**
         * Creates a new SimulationRobotProperties instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SimulationRobotProperties instance
         */
        public static create(properties?: proto.ISimulationRobotProperties): proto.SimulationRobotProperties;

        /**
         * Encodes the specified SimulationRobotProperties message. Does not implicitly {@link proto.SimulationRobotProperties.verify|verify} messages.
         * @param message SimulationRobotProperties message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISimulationRobotProperties, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SimulationRobotProperties message, length delimited. Does not implicitly {@link proto.SimulationRobotProperties.verify|verify} messages.
         * @param message SimulationRobotProperties message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISimulationRobotProperties, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SimulationRobotProperties message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SimulationRobotProperties
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SimulationRobotProperties;

        /**
         * Decodes a SimulationRobotProperties message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SimulationRobotProperties
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SimulationRobotProperties;

        /**
         * Verifies a SimulationRobotProperties message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SimulationRobotProperties message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SimulationRobotProperties
         */
        public static fromObject(object: { [k: string]: any }): proto.SimulationRobotProperties;

        /**
         * Creates a plain object from a SimulationRobotProperties message. Also converts values to other types if specified.
         * @param message SimulationRobotProperties
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SimulationRobotProperties, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SimulationRobotProperties to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SimulationRobotProperties
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SimulationConfiguration. */
    interface ISimulationConfiguration {

        /** SimulationConfiguration ballLocation */
        ballLocation?: (proto.ISimulationBallLocation|null);

        /** SimulationConfiguration robotLocations */
        robotLocations?: (proto.ISimulationRobotLocation[]|null);

        /** SimulationConfiguration robotProperties */
        robotProperties?: (proto.ISimulationRobotProperties[]|null);

        /** SimulationConfiguration simulationSpeed */
        simulationSpeed?: (number|null);

        /** SimulationConfiguration visionPort */
        visionPort?: (number|null);
    }

    /** Represents a SimulationConfiguration. */
    class SimulationConfiguration implements ISimulationConfiguration {

        /**
         * Constructs a new SimulationConfiguration.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISimulationConfiguration);

        /** SimulationConfiguration ballLocation. */
        public ballLocation?: (proto.ISimulationBallLocation|null);

        /** SimulationConfiguration robotLocations. */
        public robotLocations: proto.ISimulationRobotLocation[];

        /** SimulationConfiguration robotProperties. */
        public robotProperties: proto.ISimulationRobotProperties[];

        /** SimulationConfiguration simulationSpeed. */
        public simulationSpeed: number;

        /** SimulationConfiguration visionPort. */
        public visionPort: number;

        /**
         * Creates a new SimulationConfiguration instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SimulationConfiguration instance
         */
        public static create(properties?: proto.ISimulationConfiguration): proto.SimulationConfiguration;

        /**
         * Encodes the specified SimulationConfiguration message. Does not implicitly {@link proto.SimulationConfiguration.verify|verify} messages.
         * @param message SimulationConfiguration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISimulationConfiguration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SimulationConfiguration message, length delimited. Does not implicitly {@link proto.SimulationConfiguration.verify|verify} messages.
         * @param message SimulationConfiguration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISimulationConfiguration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SimulationConfiguration message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SimulationConfiguration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SimulationConfiguration;

        /**
         * Decodes a SimulationConfiguration message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SimulationConfiguration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SimulationConfiguration;

        /**
         * Verifies a SimulationConfiguration message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SimulationConfiguration message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SimulationConfiguration
         */
        public static fromObject(object: { [k: string]: any }): proto.SimulationConfiguration;

        /**
         * Creates a plain object from a SimulationConfiguration message. Also converts values to other types if specified.
         * @param message SimulationConfiguration
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SimulationConfiguration, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SimulationConfiguration to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SimulationConfiguration
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Namespace RoboCup2014Legacy. */
    namespace RoboCup2014Legacy {

        /** Namespace Geometry. */
        namespace Geometry {

            /** Properties of a SSL_GeometryFieldSize. */
            interface ISSL_GeometryFieldSize {

                /** SSL_GeometryFieldSize lineWidth */
                lineWidth: number;

                /** SSL_GeometryFieldSize fieldLength */
                fieldLength: number;

                /** SSL_GeometryFieldSize fieldWidth */
                fieldWidth: number;

                /** SSL_GeometryFieldSize boundaryWidth */
                boundaryWidth: number;

                /** SSL_GeometryFieldSize refereeWidth */
                refereeWidth: number;

                /** SSL_GeometryFieldSize goalWidth */
                goalWidth: number;

                /** SSL_GeometryFieldSize goalDepth */
                goalDepth: number;

                /** SSL_GeometryFieldSize goalWallWidth */
                goalWallWidth: number;

                /** SSL_GeometryFieldSize centerCircleRadius */
                centerCircleRadius: number;

                /** SSL_GeometryFieldSize defenseRadius */
                defenseRadius: number;

                /** SSL_GeometryFieldSize defenseStretch */
                defenseStretch: number;

                /** SSL_GeometryFieldSize freeKickFromDefenseDist */
                freeKickFromDefenseDist: number;

                /** SSL_GeometryFieldSize penaltySpotFromFieldLineDist */
                penaltySpotFromFieldLineDist: number;

                /** SSL_GeometryFieldSize penaltyLineFromSpotDist */
                penaltyLineFromSpotDist: number;
            }

            /** Represents a SSL_GeometryFieldSize. */
            class SSL_GeometryFieldSize implements ISSL_GeometryFieldSize {

                /**
                 * Constructs a new SSL_GeometryFieldSize.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize);

                /** SSL_GeometryFieldSize lineWidth. */
                public lineWidth: number;

                /** SSL_GeometryFieldSize fieldLength. */
                public fieldLength: number;

                /** SSL_GeometryFieldSize fieldWidth. */
                public fieldWidth: number;

                /** SSL_GeometryFieldSize boundaryWidth. */
                public boundaryWidth: number;

                /** SSL_GeometryFieldSize refereeWidth. */
                public refereeWidth: number;

                /** SSL_GeometryFieldSize goalWidth. */
                public goalWidth: number;

                /** SSL_GeometryFieldSize goalDepth. */
                public goalDepth: number;

                /** SSL_GeometryFieldSize goalWallWidth. */
                public goalWallWidth: number;

                /** SSL_GeometryFieldSize centerCircleRadius. */
                public centerCircleRadius: number;

                /** SSL_GeometryFieldSize defenseRadius. */
                public defenseRadius: number;

                /** SSL_GeometryFieldSize defenseStretch. */
                public defenseStretch: number;

                /** SSL_GeometryFieldSize freeKickFromDefenseDist. */
                public freeKickFromDefenseDist: number;

                /** SSL_GeometryFieldSize penaltySpotFromFieldLineDist. */
                public penaltySpotFromFieldLineDist: number;

                /** SSL_GeometryFieldSize penaltyLineFromSpotDist. */
                public penaltyLineFromSpotDist: number;

                /**
                 * Creates a new SSL_GeometryFieldSize instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns SSL_GeometryFieldSize instance
                 */
                public static create(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Encodes the specified SSL_GeometryFieldSize message. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize.verify|verify} messages.
                 * @param message SSL_GeometryFieldSize message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified SSL_GeometryFieldSize message, length delimited. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize.verify|verify} messages.
                 * @param message SSL_GeometryFieldSize message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SSL_GeometryFieldSize
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns SSL_GeometryFieldSize
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Verifies a SSL_GeometryFieldSize message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a SSL_GeometryFieldSize message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns SSL_GeometryFieldSize
                 */
                public static fromObject(object: { [k: string]: any }): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Creates a plain object from a SSL_GeometryFieldSize message. Also converts values to other types if specified.
                 * @param message SSL_GeometryFieldSize
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this SSL_GeometryFieldSize to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for SSL_GeometryFieldSize
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }

            /** Properties of a SSL_GeometryData. */
            interface ISSL_GeometryData {

                /** SSL_GeometryData field */
                field: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize;

                /** SSL_GeometryData calib */
                calib?: (proto.ISSL_GeometryCameraCalibration[]|null);
            }

            /** Represents a SSL_GeometryData. */
            class SSL_GeometryData implements ISSL_GeometryData {

                /**
                 * Constructs a new SSL_GeometryData.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData);

                /** SSL_GeometryData field. */
                public field: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize;

                /** SSL_GeometryData calib. */
                public calib: proto.ISSL_GeometryCameraCalibration[];

                /**
                 * Creates a new SSL_GeometryData instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns SSL_GeometryData instance
                 */
                public static create(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Encodes the specified SSL_GeometryData message. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryData.verify|verify} messages.
                 * @param message SSL_GeometryData message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified SSL_GeometryData message, length delimited. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryData.verify|verify} messages.
                 * @param message SSL_GeometryData message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SSL_GeometryData message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SSL_GeometryData
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Decodes a SSL_GeometryData message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns SSL_GeometryData
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Verifies a SSL_GeometryData message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a SSL_GeometryData message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns SSL_GeometryData
                 */
                public static fromObject(object: { [k: string]: any }): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Creates a plain object from a SSL_GeometryData message. Also converts values to other types if specified.
                 * @param message SSL_GeometryData
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.RoboCup2014Legacy.Geometry.SSL_GeometryData, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this SSL_GeometryData to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for SSL_GeometryData
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }
        }

        /** Namespace Wrapper. */
        namespace Wrapper {

            /** Properties of a SSL_WrapperPacket. */
            interface ISSL_WrapperPacket {

                /** SSL_WrapperPacket detection */
                detection?: (proto.ISSL_DetectionFrame|null);

                /** SSL_WrapperPacket geometry */
                geometry?: (proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData|null);
            }

            /** Represents a SSL_WrapperPacket. */
            class SSL_WrapperPacket implements ISSL_WrapperPacket {

                /**
                 * Constructs a new SSL_WrapperPacket.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket);

                /** SSL_WrapperPacket detection. */
                public detection?: (proto.ISSL_DetectionFrame|null);

                /** SSL_WrapperPacket geometry. */
                public geometry?: (proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData|null);

                /**
                 * Creates a new SSL_WrapperPacket instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns SSL_WrapperPacket instance
                 */
                public static create(properties?: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Encodes the specified SSL_WrapperPacket message. Does not implicitly {@link proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket.verify|verify} messages.
                 * @param message SSL_WrapperPacket message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified SSL_WrapperPacket message, length delimited. Does not implicitly {@link proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket.verify|verify} messages.
                 * @param message SSL_WrapperPacket message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SSL_WrapperPacket message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SSL_WrapperPacket
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Decodes a SSL_WrapperPacket message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns SSL_WrapperPacket
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Verifies a SSL_WrapperPacket message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a SSL_WrapperPacket message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns SSL_WrapperPacket
                 */
                public static fromObject(object: { [k: string]: any }): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Creates a plain object from a SSL_WrapperPacket message. Also converts values to other types if specified.
                 * @param message SSL_WrapperPacket
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this SSL_WrapperPacket to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for SSL_WrapperPacket
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }
        }
    }
}
