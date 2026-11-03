package first.robot.util;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableType;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.PubSubOptions;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableFloat;
import org.wpilib.tunable.TunableInt;
import org.wpilib.tunable.TunableLong;
import org.wpilib.tunable.TunableOption;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.struct.Struct;
import org.wpilib.util.struct.StructBuffer;

public class NTEntry {
    private NetworkTableEntry wrappedEntry;
    private NTable child;
    private NTable ntable;
    private TunableBase tunable; // Tunable<T> where T matches this.type
    private NetworkTableType type;

    private static HashMap<Class<?>, Struct<?>> structCache = new HashMap<>();

    private String name;

    NTEntry(NTable table, String name) {
        int entryHandle = NetworkTablesJNI.getEntry(NTable.instance.getHandle(), table.getPath() + "/" + name);
        var entry = new NetworkTableEntry(NTable.instance, entryHandle);
        this(table, name, entry.getType(), entry);
    }

    NTEntry(NTable table, String name, NetworkTableEntry entry) { this(table, name, entry.getType(), entry); }

    NTEntry(NTable table, String name, NetworkTableType type, String typeString, PubSubOptions opts) {
        int topic = NetworkTablesJNI.getTopic(NTable.instance.getHandle(), table.getPath() + "/" + name);
        var entry =
            new NetworkTableEntry(NTable.instance, NetworkTablesJNI.getEntry(topic, type.getValue(), typeString, opts));
        this(table, name, type, entry);
    }

    private static PubSubOptions build(PubSubOption... opts) {
        if (opts.length == 0) return null;
        else return new PubSubOptions(opts);
    }

    NTEntry(NTable table, String name, NetworkTableType type, String typeString, PubSubOption... opts) {
        this(table, name, type, typeString, build(opts));
    }

    NTEntry(NTable table, String name, NetworkTableType type, NetworkTableEntry entry) {
        this.ntable = table;
        this.name = name;
        this.type = type;
        this.wrappedEntry = entry;

        if (type != NetworkTableType.UNASSIGNED) {
            NetworkTableValue val = entry.getValue();
            this.tunable = Tunable.createConfig(val.getValue(), TunableConfig.of(TunableOption.ALWAYS_GET));
        } else {
            this.tunable = null;
        }

        if (this.ntable.entries.containsKey(name)) {
            throw new IllegalStateException("tried to create a duplicate NTable.entry");
        }
    }

    /** {@return the this.ntable is an entry of} */
    public NTable getContainingNTable() { return this.ntable; }

    /** {@return the {@link NetworkTableEntry} representing this entry} */
    public NetworkTableEntry getRawEntry() { return this.wrappedEntry; }

    /** {@return the name of this entry} */
    public String getName() { return this.name; }

    /** {@return the full path of this entry} */
    public String getPath() { return this.ntable.getPath() + "/" + this.name; }

    /** {@return the {@link Tunable} representing this entry} */
    public TunableBase getTunable() { return this.tunable; }

    /** {@return this.getContainingNTable().sub(this.name())} */
    public NTable getAsNTable() {
        if (this.child == null) this.child = new NTable(this.ntable.table.getSubTable(this.name), this.ntable);
        return this.child;
    }

    /** {@return this entry's handle} */
    public int getHandle() { return this.getRawEntry().getHandle(); }

    public NetworkTableType getType() {
        NetworkTableType retrievedType = this.wrappedEntry.getType();
        if (this.type.equals(NetworkTableType.UNASSIGNED)) {
            this.type = retrievedType;
            return retrievedType;
        } else if (!this.type.equals(retrievedType)) {
            throw new IllegalStateException(
                getPath() + " retrieved type " + retrievedType + " does not match stored type " + this.type
            );
        } else {
            return this.type;
        }
    }

    /** {@return whether this entry exists} */
    public boolean exists() { return this.wrappedEntry.exists(); }

    /**
     * Publishes a value of any accepted type to the NetworkTable.
     *
     * <p>
     * The type of the value is determined from the runtime type of the object
     * passed to the parameter {@code value}. This function acts as a catch-all for
     * anything that can be posted to NetworkTables, in the spirit of this class's
     * 'set and forget' philosophy.
     *
     * <p>
     * If the type is a so-called 'simple' type, it is passed to
     * {@link #setSimple(Object)}. See the end of this function's documentation for
     * a discussion of the term 'simple'.
     *
     * <p>
     * Otherwise, the type is checked to see whether it has a registered
     * {@code Struct<T>} associated with its class type. If so, it is passed to
     * {@link #setStruct(Struct)}. set also attempts to automatically detect if the
     * passed object's class type has a static member named 'struct', and if so,
     * that struct is registered for the passed class type and the object is then
     * passed to {@link #setStruct(Struct)}.
     *
     * <p>
     * If the object is of array type, it is checked whether it is an array of
     * struct-serializable objects. If that is the case, then it is serialized as an
     * array of structs.
     *
     * <p>
     * If none of the above cases apply, a warning is printed and the object is
     * ignored.
     *
     * <p>
     * The following types are considered 'simple':
     *
     * <ul>
     * <li>{@code Boolean}</li>
     * <li>{@code Float}</li>
     * <li>{@code Long}</li>
     * <li>{@code Double}</li>
     * <li>{@code String}</li>
     * </ul>
     *
     * Arrays of any of the above, as well as arrays of their respective primitive
     * types (except for String, which does not have an associated primitive type)
     * are also considered 'simple'. For example, the following types are 'simple':
     * {@code String[]}, {@code double[]}, {@code int}, etc. Note that primitive
     * types such as {@code int} automatically get 'boxed' into their corresponding
     * {@link Object} type, such as {@link Integer} in the case of int. Therefore,
     * passing a primitive to the value parameter of this function will work as
     * expected.
     *
     * <p>
     * There are two other types considered 'simple': {@code byte[]} and
     * {@code Byte[]}. These are classified as 'raw' data and are typically used to
     * send structs across NetworkTables (see {@link #setStruct(Struct)}).
     *
     * <p>
     * Furthermore, any numeric primitive box type that extends {@code Number} is
     * also considered primitive and will be sent as a double, except for the
     * non-double numerics mentioned above, {@code Float} and {@code Long}. For
     * example, this function will accept a {@code Byte} (not an array of byte; that
     * would fall under the previous paragraph!) and send it as a double.
     *
     * @param value the Object to publish
     *
     * @bug This function does not properly handle classes that are only
     *      de/serializable with Protobuffers.
     */
    public <T> void set(T value) {
        // if the value is a simple type, publish it simply.
        if (NetworkTableEntry.isValidDataType(value)) {
            setSimple(value);
            return;
        }

        // getStructForObject returns null when the object doesn't have an associated
        // struct for its class. using this, we can verify that the object is a struct
        // type.
        Struct<?> possibleStruct = getStructForType(value.getClass());
        if (possibleStruct != null) {
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>) possibleStruct;
            setStruct(value, casted);
            return;
        }

        // if the object is an array, check whether it is an array of
        // struct-serializable objects.
        if (value.getClass().isArray() && value instanceof Object[] casted) {
            possibleStruct = getStructForType(casted.getClass().getComponentType());
            if (possibleStruct != null) {
                // if we got this far, we know that castedStruct is Struct<T> where T is the
                // component type of value. due to Java generics being a pile of type-erasing
                // bullshit, we can cast everything to be in terms of Object and it should just
                // work.
                @SuppressWarnings("unchecked") Struct<Object> castedStruct = (Struct<Object>) possibleStruct;
                setStructArray(casted, castedStruct);
                return;
            }
        }

        // if the object is TelemetryLoggable, log it with the Telemetry API
        if (value instanceof TelemetryLoggable telemetryLoggable) {
            this.ntable.telemetryTable.log(this.name, telemetryLoggable);
        }

        // If none of the above cases apply, print a warning.
        DriverStationErrors.reportError(
            "NTable: Could not publish value of type " + value.getClass().getCanonicalName() + " to entry " +
                getPath() + ": it is not supported.",
            false
        );
    }

    /**
     * Publishes a value of any accepted 'simple' type to the NetworkTable.
     *
     * <p>
     * See the documentation for {@link #set(Object)} for a discussion of 'simple'
     * types.
     *
     * <p>
     * The type of the value is determined from the runtime type of the object. If
     * the type is not supported by NetworkTables, a warning is printed to
     * DriverStation and nothing is published. If the type differs from the type
     * previously posted to this entry in this NTable, a warning is printed and
     * nothing is published.
     *
     * <p>
     * Note that primitives such as `double` and `boolean` are automatically boxed
     * by Java into an {@link Object}, such as {@link Double} and {@link Boolean}.
     * Primitive arrays can also be passed without any necessary client-side code as
     * they are {@link Object}s.
     *
     * @param value the Object to publish
     */
    public void setSimple(Object value) {
        if (!NetworkTableEntry.isValidDataType(value)) {
            DriverStationErrors.reportWarning(
                "NTable entry " + getPath() + " has invalid type; the passed object is of type " +
                    value.getClass().getName(),
                true
            );
            return;
        }
        this.wrappedEntry.setValue(value);
    }

    /**
     * Uses runtime reflection on classType to retrieve its static 'struct' member,
     * if it exists.
     *
     * <p>
     * If in any case the associated struct for this class type cannot be resolved,
     * null is returned. This function also caches results it resolves to avoid
     * repeated reflection. The cache key is the passed class type. See
     * {@link #structCacheor the cache.
     *
     * @param classType the class type to get the struct for
     *
     * @return the struct for the given class type
     */
    private <T> Struct<T> getStructForType(Class<?> classType) {
        if (classType == null) {
            DriverStationErrors.reportWarning("null class type passed trying to retrieve struct", true);
            return null;
        }

        if (structCache.containsKey(classType)) {
            // if the struct has already been extracted and cached by the rest of the
            // function, use it
            Struct<?> struct = structCache.get(classType);
            if (!struct.getTypeClass().isAssignableFrom(classType)) {
                DriverStationErrors.reportError(
                    "tried to publish a " + classType.getName() + ", but a struct of type " +
                        struct.getTypeClass().getName() + " had already been registered",
                    true
                );
                return null;
            }
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>) struct;
            return casted;
        }

        // assuming that the struct is stored in T.struct, as is the convention, get it
        // using runtime reflection (exciting!)
        try {
            Field field = classType.getField("struct");
            if (!Modifier.isStatic(field.getModifiers())) {
                return null;
            }
            Object possibleStruct = field.get(null);
            if (!(possibleStruct instanceof Struct<?> struct) || !struct.getTypeClass().isAssignableFrom(classType)) {
                return null;
            }
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>) struct;
            structCache.put(classType, struct);
            return casted;

        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Publishes a struct to the NetworkTable.
     *
     * <p>
     * Instead of using {@link #getStructForType(Class)}, use the given Struct
     * object for serialization.
     *
     * @param value  the value to publish
     * @param struct the struct with which to serialize the value
     */
    public <T> void setStruct(T value, Struct<T> struct) {
        NetworkTableInstance.getDefault().addSchema(struct);
        StructBuffer<T> buf = StructBuffer.create(struct);
        synchronized (buf) {
            ByteBuffer raw = buf.write(value);
            publishRawBuffer(raw, struct.getTypeString());
        }
    }

    /**
     * Publishes an array of structs to the NetworkTable.
     *
     * <p>
     * Instead of using {@link #getStructForType(Class)}, use the given Struct
     * object for serialization.
     *
     * @param values the values to publish
     * @param struct the struct with which to serialize the values
     */
    public <T> void setStructArray(T[] values, Struct<T> struct) {
        NetworkTableInstance.getDefault().addSchema(struct);
        publishRawBuffer(StructBuffer.create(struct).writeArray(values), struct.getTypeString());
    }

    /**
     * Publishes a struct to the NetworkTable.
     *
     * @see #getStructForType(Class)
     * @param value the value to publish
     */
    public <T> void setStruct(T value) {
        Struct<T> struct = getStructForType(value.getClass());
        if (struct != null) {
            setStruct(value, struct);
        }
    }

    /**
     * Publishes an array of structs to the NetworkTable.
     *
     * @see #getStructForType(Class)
     * @param values the values to publish
     */
    public <T> void setStructs(T[] values) {
        Struct<T> struct = getStructForType(values.getClass().getComponentType());
        if (struct != null) {
            setStructArray(values, struct);
        }
    }

    /** Publishes a ByteBuffer to the NetworkTable. */
    private void publishRawBuffer(ByteBuffer buffer, String typeString) {
        NetworkTablesJNI.setRaw(this.getHandle(), NetworkTablesJNI.now(), buffer, 0, buffer.position());
    }

    /**
     * Retrieves a value of any accepted type from NetworkTables.
     *
     * <p>
     * The type of the value is determined from the runtime type of the object
     * passed to the parameter {@code defaultValue}. This function acts as a
     * catch-all for anything that can be retrieved from NetworkTables, in the
     * spirit of this class's 'set and forget' philosophy.
     *
     * <p>
     * If the type is a so-called 'simple' type, this function defers to
     * {@link #getSimple(NetworkTableType)}. See the end of this function's
     * documentation for a discussion of the term 'simple'.
     *
     * <p>
     * If the type is a {@link Sendable}, this function returns the stored sendable
     * as was last set under this name. In other words, setting a sendable then
     * getting it should return the exact same object back to the caller.
     *
     * <p>
     * Otherwise, the type is checked to see whether it has a registered
     * {@code Struct<T>} associated with its class type. If so, this function defers
     * to {@link #getStruct(Struct)}. set also attempts to automatically detect if
     * the passed object's class type has a static member named 'struct', and if so,
     * that struct is registered for the passed class type and the function then
     * defers to {@link #getStruct(Struct)}.
     *
     * <p>
     * If the object is of array type, it is checked whether it is an array of
     * struct-serializable objects. If that is the case, then it is deserialized as
     * an array of structs by deferring to {@link #getStructArray(Struct)}.
     *
     * <p>
     * If none of the above cases apply, a warning is printed and the defalut value
     * is returned.
     *
     * <p>
     * The following types are considered 'simple':
     *
     * <ul>
     * <li>{@code Boolean}</li>
     * <li>{@code Float}</li>
     * <li>{@code Long}</li>
     * <li>{@code Double}</li>
     * <li>{@code String}</li>
     * </ul>
     *
     * Arrays of any of the above, as well as arrays of their respective primitive
     * types (except for String, which does not have an associated primitive type)
     * are also considered 'simple'. For example, the following types are 'simple':
     * {@code String[]}, {@code double[]}, {@code int}, etc. Note that primitive
     * types such as {@code int} automatically get 'boxed' into their corresponding
     * {@link Object} type, such as {@link Integer} in the case of int. Therefore,
     * passing a primitive to the value parameter of this function will work as
     * expected.
     *
     * <p>
     * There are two other types considered 'simple': {@code byte[]} and
     * {@code Byte[]}. These are classified as 'raw' data and are typically used to
     * send structs across NetworkTables (see {@link #setStruct(String, Struct)}).
     *
     * <p>
     * Furthermore, any numeric primitive box type that extends {@code Number} is
     * also considered primitive and will be sent as a double, except for the
     * non-double numerics mentioned above, {@code Float} and {@code Long}. For
     * example, this function will accept a {@code Byte} (not an array of byte;
     * that would fall under the previous paragraph!) and send it as a double.
     *
     * @param defaultValue the default value to return if the entry does not exist
     *                     or is invalid
     *
     * @bug This function does not properly handle classes that are only
     *      de/serializable with Protobuffers.
     */
    public <T> T get(T defaultValue) {
        // if the value is a simple type, retrieve it and attempt to cast it to the
        // requested type.
        if (NetworkTableEntry.isValidDataType(defaultValue)) {
            NetworkTableValue retrieved = this.wrappedEntry.getValue();
            Class<?> classType = defaultValue.getClass();
            if (!classType.isInstance(retrieved.getValue())) {
                return defaultValue;
            }
            @SuppressWarnings("unchecked") T value = (T) defaultValue.getClass().cast(retrieved.getValue());
            return value;
        }

        // getStructForObject returns null when the object doesn't have an associated
        // struct for its class. Using this, we can verify that the object is a struct
        // type.
        Struct<?> possibleStruct = getStructForType(defaultValue.getClass());
        if (possibleStruct != null) {
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>) possibleStruct;
            T result = getStruct(casted);
            if (result == null) {
                return defaultValue;
            }
            return result;
        }

        // if the object is an array, check whether it is an array of
        // struct-deserializable objects.
        if (defaultValue.getClass().isArray() && defaultValue instanceof Object[] casted) {
            Struct<?> possibleStruct2 = getStructForType(casted.getClass().getComponentType());
            if (possibleStruct2 != null) {
                // we already know that T is an array type, so it's safe to assume that and cast
                // directly to it. note that the 'T' in getStructArray is different from the T
                // here, and is in fact the value-type of the T here (meaning the T of #get is
                // of type E[] where E is referred to as T within getStructArray)
                @SuppressWarnings("unchecked") T result = (T) getStructArray(possibleStruct2);
                if (result == null) {
                    return defaultValue;
                }
                return result;
            }
        }

        // if none of the above cases apply, print a warning.
        DriverStationErrors.reportError(
            "NTable: Could not retrieve value of type " + defaultValue.getClass().getName() + " from entry " +
                this.name + ": it is not supported.",
            true
        );
        return defaultValue;
    }

    /**
     * Gets the value of the given type from the NetworkTable.
     *
     * <p>
     * If the requested type differs from the type retrieved from the entry, a
     * warning is printed and a value of the current entry's type is returned. If no
     * value has been published under this name, returns a NetworkTableValue with
     * type kUnassigned.
     *
     * @param type the type of the entry
     *
     * @return the requested value as a {@link NetworkTableValue}
     */
    public NetworkTableValue getSimple(NetworkTableType type) { return this.wrappedEntry.getValue(); }

    /** @see #getSimple(NetworkTableType) */
    public double getDouble() { return getSimple(NetworkTableType.DOUBLE).getDouble(); }

    /** @see #getSimple(NetworkTableType) */
    public boolean getBoolean() { return getSimple(NetworkTableType.BOOLEAN).getBoolean(); }

    /** @see #getSimple(NetworkTableType) */
    public String getString() { return getSimple(NetworkTableType.STRING).getString(); }

    /** @see #getSimple(NetworkTableType) */
    public long getInt() { return getSimple(NetworkTableType.INTEGER).getInteger(); }

    /** @see #getSimple(NetworkTableType) */
    public float getLong() { return getSimple(NetworkTableType.FLOAT).getFloat(); }

    /** @see #getSimple(NetworkTableType) */
    public byte[] getAsRaw() { return getSimple(NetworkTableType.RAW).getRaw(); }

    /** @see #getSimple(NetworkTableType) */
    public double[] getDoubleArray() { return getSimple(NetworkTableType.DOUBLE_ARRAY).getDoubleArray(); }

    /** @see #getSimple(NetworkTableType) */
    public boolean[] getBooleanArray() { return getSimple(NetworkTableType.BOOLEAN_ARRAY).getBooleanArray(); }

    /** @see #getSimple(NetworkTableType) */
    public String[] getStringArray() { return getSimple(NetworkTableType.STRING_ARRAY).getStringArray(); }

    /** @see #getSimple(NetworkTableType) */
    public long[] getIntArray() { return getSimple(NetworkTableType.INTEGER_ARRAY).getIntegerArray(); }

    /** @see #getSimple(NetworkTableType) */
    public float[] getFloatArray() { return getSimple(NetworkTableType.FLOAT_ARRAY).getFloatArray(); }

    /**
     * Attempts to retrieve and deserialize a struct under the given path in
     * NetworkTables.
     *
     * <p>
     * If the given entry name does not currently store a "raw" value, this function
     * will error out like {@link #getAsRaw()}. Further, if the value can be
     * retrieved but not unpacked, a warning is reported to the DriverStation.
     *
     * @param struct the struct to use for deserialization
     *
     * @return the unpacked struct
     */
    public <T> T getStruct(Struct<T> struct) {
        byte[] raw = getAsRaw();
        if (raw.length == 0) {
            return null;
        }
        try {
            StructBuffer<T> buffer = StructBuffer.create(struct);
            return buffer.read(raw);
        } catch (RuntimeException e) {
            DriverStationErrors.reportWarning(
                "NTable entry " + getPath() + " could not be unpacked: " + e.getMessage(), true
            );
            return null;
        }
    }

    /**
     * Attempts to retrieve and deserialize an array of struct objects under the
     * given path in NetworkTables.
     *
     * <p>
     * If the given entry name does not currently store a "raw" value, this function
     * will error out like {@link #getAsRaw()}. Further, if the value can be
     * retrieved but not unpacked, a warning is reported to the DriverStation.
     *
     * @param struct the struct to use for deserialization
     *
     * @return the unpacked struct
     */
    public <T> T[] getStructArray(Struct<T> struct) {
        byte[] raw = getAsRaw();
        if (raw.length == 0) {
            return null;
        }
        try {
            StructBuffer<T> buffer = StructBuffer.create(struct);
            return buffer.readArray(raw);
        } catch (RuntimeException e) {
            DriverStationErrors.reportWarning(
                "NTable entry " + getPath() + " could not be unpacked: " + e.getMessage(), true
            );
            return null;
        }
    }

    /**
     * Creates a tunable for the named entry with the given getter, setter,
     * classType, and config.
     *
     * @param getter      function called when NT wants to update the tunable's
     *                    value
     * @param setCallback function called when the tunable is changed in NT
     * @param classType   relevant type
     * @param config      config of the tunable
     *
     * @return the Tunable
     */
    public <T> Tunable<T>
    tunable(Supplier<T> getter, Consumer<T> setCallback, Class<T> classType, TunableConfig config) {
        Tunable<T> res = Tunable.createConfig(getter, setCallback, classType, config);
        TunableRegistry.publish(getPath(), res);
        return res;
    }

    public <T> Tunable<T> tunable(Supplier<T> getter, Consumer<T> setCallback, Class<T> classType) {
        return tunable(getter, setCallback, classType, null);
    }

    public <T> Tunable<T> tunable(T initialValue, TunableConfig config) {
        Tunable<T> res = Tunable.createConfig(initialValue, config);
        TunableRegistry.publish(getPath(), res);
        return res;
    }

    public <T> Tunable<T> tunable(T initialValue) { return tunable(initialValue, null); }

    public <T> Tunable<T> tunable(Class<T> classType, TunableConfig config) {
        Tunable<T> res = Tunable.createNullConfig(classType, config);
        TunableRegistry.publish(getPath(), res);
        return res;
    }

    public <T> Tunable<T> tunable(Class<T> classType) { return tunable(classType, null); }

    /**
     * Returns a tunable of the named entry with the given Tunable type, initial
     * value, and value type.
     *
     * <p>
     * T should look like TunableU, e.g. U = Boolean -> T = TunableBoolean
     *
     * @param <T>          type of the returned tunable (T.class == classType)
     * @param <U>          type of what the tunable tracks (U.class == valueType)
     * @param classType    type to return
     * @param initialValue initial value for the tunable; before an update from NT,
     *                     the initial value is what will be
     *                     returned by .get
     * @param valueType    the type of the value tracked by the tunable you want to
     *                     create
     * @return the tunable
     */
    private <T extends TunableBase, U> T tunableOf(Class<T> classType, U initialValue, Class<U> valueType) {
        try {
            Method method = classType.getMethod("createConfig", valueType, TunableConfig.class);
            @SuppressWarnings("unchecked")
            T res = (T) method.invoke(null, initialValue, TunableConfig.of(TunableOption.ALWAYS_GET));
            TunableRegistry.publish(getPath(), res);
            return res;
        } catch (ReflectiveOperationException e) {
            DriverStationErrors.reportWarning(
                "some error occurred while making a tunable of type " + classType.getCanonicalName(), true
            );
            DriverStationErrors.reportWarning("error: " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    /** {@return a {@link TunableInt} for this entry with the given initial value} */
    public TunableInt tunableInt(int init) { return tunableOf(TunableInt.class, init, int.class); }

    /** {@return a {@link TunableLong} for this entry with the given initial value} */
    public TunableLong tunableLong(long init) { return tunableOf(TunableLong.class, init, long.class); }

    /** {@return a {@link TunableFloat} for this entry with the given initial value} */
    public TunableFloat tunableFloat(float init) { return tunableOf(TunableFloat.class, init, float.class); }

    /** {@return a {@link TunableDouble} for this entry with the given initial value} */
    public TunableDouble tunableDouble(double init) { return tunableOf(TunableDouble.class, init, double.class); }

    /** {@return a {@link TunableBoolean} for this entry with the given initial value} */
    public TunableBoolean tunableBoolean(boolean init) { return tunableOf(TunableBoolean.class, init, boolean.class); }

    /** {@return a {@link TunableInt} for this entry} */
    public TunableInt tunableInt() { return tunableOf(TunableInt.class, 0, int.class); }

    /** {@return a {@link TunableLong} for this entry} */
    public TunableLong tunableLong() { return tunableOf(TunableLong.class, 0l, long.class); }

    /** {@return a {@link TunableFloat} for this entry} */
    public TunableFloat tunableFloat() { return tunableOf(TunableFloat.class, 0f, float.class); }

    /** {@return a {@link TunableDouble} for this entry} */
    public TunableDouble tunableDouble() { return tunableOf(TunableDouble.class, 0d, double.class); }

    /** {@return a {@link TunableBoolean} for this entry} */
    public TunableBoolean tunableBoolean() { return tunableOf(TunableBoolean.class, false, boolean.class); }

    /**
     * Register consumer to be called every time the entry under name changes.
     *
     * @param callback  callback to call on change
     * @param classType the type of the entry you want to listen to
     */
    public <T> void listen(Consumer<T> callback, Class<T> classType) {
        tunable(null, callback, classType, TunableConfig.of(TunableOption.GET_ON_CHANGE));
    }

    public NTEntry persists(boolean shouldPersist) {
        if (shouldPersist) this.wrappedEntry.setPersistent();
        else this.wrappedEntry.clearPersistent();
        return this;
    };

    public boolean persists() { return this.wrappedEntry.isPersistent(); }

    public <T> NTEntry defaultTo(T value) {
        if (!this.exists()) this.set(value);
        return this;
    }

    public <T> NTEntry ensure(T value) { return this.defaultTo(value).persists(true); }
}
