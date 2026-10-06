package first.robot.util;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableType;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.NetworkTablesJNI;
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

/**
 * A {@link NetworkTable}.
 *
 * <p>
 * This class represents a NetworkTable. Its purpose is to make the
 * NetworkTables API nicer to use, as well as adding consistent caching logic
 * for entries and tables to hopefully permanently eliminate continuously making
 * publishers or tables. It also provides factory functions for integration with
 * 2027's Tunable and Telemetry APIs.
 *
 * <p>
 * Note that one invariant of this class is that every NetworkTable is
 * represented by exactly one NTable. This ensures that
 * each {@link NetworkTableEntry} managed by a NTable is only created once,
 * allowing much more flexibility in client code.
 */
public class NTable {
    private static final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static final NTable root = new NTable(instance.getTable(""), null);

    private final NetworkTable table;
    private final NTable parent;

    /** {@return the parent of this NTable, or null if this == NTable.root()} */
    public NTable getParent() { return parent; }

    /** {@return the underlying {@link NetworkTable}} */
    public NetworkTable getTable() { return table; }

    /** {@return the path of this NTable} */
    public String getPath() { return this.table.getPath(); }

    private HashMap<String, NTable> subTables = new HashMap<>();
    private HashMap<String, NetworkTableEntry> entries = new HashMap<>();

    private NTable(NetworkTable table, NTable parent) {
        this.table = table;
        this.parent = parent;
        long depth = this.table.getPath().chars().filter(c -> c == '/').count();
        if (depth > 50) {
            DriverStationErrors.reportWarning("very long NTable of depth " + depth +
                                                  " created! be careful. its path is " + this.table.getPath(),
                                              true);
        }

        for (String entry : this.table.getKeys()) {
            entries.put(entry, this.table.getEntry(entry));
        }
    }

    /** {@return the root NTable} */
    public static NTable root() { return root; }

    /**
     * {@return a subtable of the root NTable}
     *
     * @param name the name of the subtable
     */
    public static NTable root(String name) { return root().sub(name); }

    /**
     * {@return a subtable of this NTable}
     *
     * <p>
     * Creates the subtable if it does not already exist.
     *
     * @param name the name of the subtable
     */
    public NTable sub(String name) {
        return subTables.computeIfAbsent(name, n -> new NTable(table.getSubTable(n), this));
    }

    /**
     * {@return an entry of this NTable}
     *
     * <p>
     * Creates the entry if it does not already exist. Note that unless this entry
     * has been set, this function will see the topic as being of an unassigned
     * type. If the entry already exists with a different type than the one given in
     * type and warnOnWrongType is true, a warning will be printed.
     *
     * @param name the name of the entry
     */
    public NetworkTableEntry getEntry(String name) {
        return entries.computeIfAbsent(
            name,
            n
            -> new NetworkTableEntry(
                instance, NetworkTablesJNI.getEntry(instance.getHandle(), this.table.getPath() + "/" + name)));
    }

    public NetworkTableEntry getEntry(String name, NetworkTableType type, String typeName) {
        return entries.computeIfAbsent(
            name,
            n
            -> new NetworkTableEntry(
                instance, NetworkTablesJNI.getEntry(
                              NetworkTablesJNI.getTopic(instance.getHandle(), this.table.getPath() + "/" + name),
                              type.getValue(), typeName)));
    }

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
     * {@link #setSimple(String, Object)}. See the end of this function's
     * documentation for a discussion of the term 'simple'.
     *
     * <p>
     * Otherwise, the type is checked to see whether it has a registered
     * {@code Struct<T>} associated with its class type.
     * If so, it is passed to {@link #setStruct(String, Struct)}.
     * set also attempts to automatically detect if the passed object's class type
     * has a static member named 'struct', and if so, that struct is registered for
     * the passed class type and the object is then passed to
     * {@link #setStruct(String, Struct)}.
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
     * send structs across NetworkTables (see {@link #setStruct(String, Struct)}).
     *
     * <p>
     * Furthermore, any numeric primitive box type that extends {@code Number} is
     * also considered primitive and will be sent as a double, except for the
     * non-double numerics mentioned above, {@code Float} and {@code Long}. For
     * example, this function will accept a {@code Byte} (not an array of byte;
     * that would fall under the previous paragraph!) and send it as a double.
     *
     * @param name  the name of the entry to publish
     * @param value the Object to publish
     *
     * @bug This function does not properly handle classes that are only
     *      de/serializable with Protobuffers.
     */
    public <T> void set(String name, T value) {
        // If the value is a simple type, publish it simply.
        if (NetworkTableEntry.isValidDataType(value)) {
            setSimple(name, value);
            return;
        }

        // getStructForObject returns null when the object doesn't have an associated
        // struct for its class. Using this, we can verify that the object is a struct
        // type.
        Struct<?> possibleStruct = getStructForType(value.getClass());
        if (possibleStruct != null) {
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>)possibleStruct;
            setStruct(name, value, casted);
            return;
        }

        // If the object is an array, check whether it is an array of
        // struct-serializable objects.
        if (value.getClass().isArray() && value instanceof Object[] casted) {
            possibleStruct = getStructForType(casted.getClass().getComponentType());
            if (possibleStruct != null) {
                // If we got this far, we know that castedStruct is Struct<T> where T is the
                // component type of value. Due to Java generics being a pile of type-erasing
                // bullshit, we can cast everything to be in terms of Object and it should just
                // work.
                @SuppressWarnings("unchecked") Struct<Object> castedStruct = (Struct<Object>)possibleStruct;
                setStructArray(name, casted, castedStruct);
                return;
            }
        }

        // If none of the above cases apply, print a warning.
        DriverStationErrors.reportError("NTable: Could not publish value of type " + value.getClass().getName() +
                                            " to entry " + name + ": it is not supported.",
                                        false);
    }

    /**
     * Publishes a value of any accepted 'simple' type to the NetworkTable.
     *
     * <p>
     * See the documentation for {@link #set(String, Object)} for a discussion of
     * 'simple' types.
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
     * @param name  the name of the entry to publish
     * @param value the Object to publish
     */
    public void setSimple(String name, Object value) {
        if (!NetworkTableEntry.isValidDataType(value)) {
            DriverStationErrors.reportWarning("NTable entry " + table.getPath() + "/" + name +
                                                  " has invalid type; the passed object is of type " +
                                                  value.getClass().getName(),
                                              true);
            return;
        }
        getEntry(name).setValue(value);
    }

    /** Publishes a ByteBuffer to the NetworkTable. */
    private void publishRawBuffer(String name, ByteBuffer buffer, String typeString) {
        NetworkTablesJNI.setRaw(getEntry(name, NetworkTableType.RAW, typeString).getHandle(), NetworkTablesJNI.now(),
                                buffer, 0, buffer.position());
    }

    /**
     * A map of class types to their resolved struct objects, to avoid reflection
     */
    private HashMap<Class<?>, Struct<?>> cachedStructs = new HashMap<>();

    /**
     * Uses runtime reflection on classType to retrieve its static 'struct' member,
     * if it exists.
     *
     * <p>
     * If in any case the associated struct for this class type cannot be resolved,
     * null is returned. This function also caches results it resolves to avoid
     * repeated reflection. The cache key is the passed class type. See
     * {@link #cachedStructs} for the cache.
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

        if (cachedStructs.containsKey(classType)) {
            // if the struct has already been extracted and cached by the rest of the
            // function, use it
            Struct<?> struct = cachedStructs.get(classType);
            if (!struct.getTypeClass().isAssignableFrom(classType)) {
                DriverStationErrors.reportError("tried to publish a " + classType.getName() +
                                                    ", but a struct of type " + struct.getTypeClass().getName() +
                                                    " had already been registered for this entry in " +
                                                    table.getPath(),
                                                true);
                return null;
            }
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>)struct;
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
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>)struct;
            cachedStructs.put(classType, struct);
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
     * @param name   the name of the entry to publish
     * @param value  the value to publish
     * @param struct the struct with which to serialize the value
     */
    public <T> void setStruct(String name, T value, Struct<T> struct) {
        NetworkTableInstance.getDefault().addSchema(struct);
        StructBuffer<T> buf = StructBuffer.create(struct);
        synchronized (buf) {
            ByteBuffer raw = buf.write(value);
            publishRawBuffer(name, raw, struct.getTypeString());
        }
    }

    /**
     * Publishes an array of structs to the NetworkTable.
     *
     * <p>
     * Instead of using {@link #getStructForType(Class)}, use the given Struct
     * object for serialization.
     *
     * @param name   the name of the entry to publish
     * @param values the values to publish
     * @param struct the struct with which to serialize the values
     */
    public <T> void setStructArray(String name, T[] values, Struct<T> struct) {
        NetworkTableInstance.getDefault().addSchema(struct);
        publishRawBuffer(name, StructBuffer.create(struct).writeArray(values), struct.getTypeString());
    }

    /**
     * Publishes a struct to the NetworkTable.
     *
     * @see #getStructForType(Class)
     * @param name  the name of the entry to publish
     * @param value the value to publish
     */
    public <T> void setStruct(String name, T value) {
        Struct<T> struct = getStructForType(value.getClass());
        if (struct != null) {
            setStruct(name, value, struct);
        }
    }

    /**
     * Publishes an array of structs to the NetworkTable.
     *
     * @see #getStructForType(Class)
     * @param name   the name of the entry to publish
     * @param values the values to publish
     */
    public <T> void setStructs(String name, T[] values) {
        Struct<T> struct = getStructForType(values.getClass().getComponentType());
        if (struct != null) {
            setStructArray(name, values, struct);
        }
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
     * {@link #getSimple(String, NetworkTableType)}. See the end of this function's
     * documentation for a discussion of the term 'simple'.
     *
     * <p>
     * If the type is a {@link Sendable}, this function returns the stored sendable
     * as was last set under this name. In other words, setting a sendable then
     * getting it should return the exact same object back to the caller.
     *
     * <p>
     * Otherwise, the type is checked to see whether it has a registered
     * {@code Struct<T>} associated with its class type.
     * If so, this function defers to {@link #getStruct(String, Struct)}.
     * set also attempts to automatically detect if the passed object's class type
     * has a static member named 'struct', and if so, that struct is registered for
     * the passed class type and the function then defers to
     * {@link #getStruct(String, Struct)}.
     *
     * <p>
     * If the object is of array type, it is checked whether it is an array of
     * struct-serializable objects. If that is the case, then it is deserialized as
     * an array of structs by deferring to {@link #getStructArray(String, Struct)}.
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
     * @param name         the name of the entry to retrieve from
     * @param defaultValue the default value to return if the entry does not exist
     *                     or is invalid
     *
     * @bug This function does not properly handle classes that are only
     *      de/serializable with Protobuffers.
     */
    public <T> T get(String name, T defaultValue) {
        // If the value is a simple type, retrieve it and attempt to cast it to the
        // requested type.
        if (NetworkTableEntry.isValidDataType(defaultValue)) {
            NetworkTableValue retrieved = getEntry(name).getValue();
            Class<?> classType = defaultValue.getClass();
            if (!classType.isInstance(retrieved.getValue())) {
                return defaultValue;
            }
            @SuppressWarnings("unchecked") T value = (T)defaultValue.getClass().cast(retrieved.getValue());
            return value;
        }

        // getStructForObject returns null when the object doesn't have an associated
        // struct for its class. Using this, we can verify that the object is a struct
        // type.
        Struct<?> possibleStruct = getStructForType(defaultValue.getClass());
        if (possibleStruct != null) {
            @SuppressWarnings("unchecked") Struct<T> casted = (Struct<T>)possibleStruct;
            T result = getStruct(name, casted);
            if (result == null) {
                return defaultValue;
            }
            return result;
        }

        // If the object is an array, check whether it is an array of
        // struct-deserializable objects.
        if (defaultValue.getClass().isArray() && defaultValue instanceof Object[] casted) {
            Struct<?> possibleStruct2 = getStructForType(casted.getClass().getComponentType());
            if (possibleStruct2 != null) {
                // We already know that T is an array type, so it's safe to assume that and cast
                // directly to it. Note that the 'T' in getStructArray is different from the T
                // here,
                // and is in fact the value-type of the T here (meaning the T of #get is of type
                // E[] where E is referred to as T within getStructArray).
                @SuppressWarnings("unchecked") T result = (T)getStructArray(name, possibleStruct2);
                if (result == null) {
                    return defaultValue;
                }
                return result;
            }
        }

        // If none of the above cases apply, print a warning.
        DriverStationErrors.reportError("NTable: Could not retrieve value of type " +
                                            defaultValue.getClass().getName() + " to entry " + name +
                                            ": it is not supported.",
                                        true);
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
     * @param name the name of the entry
     * @param type the type of the entry
     *
     * @return the requested value as a {@link NetworkTableValue}
     */
    public NetworkTableValue getSimple(String name, NetworkTableType type) { return getEntry(name).getValue(); }

    /** @see #getSimple(String, NetworkTableType) */
    public double getDouble(String name) { return getSimple(name, NetworkTableType.DOUBLE).getDouble(); }

    /** @see #getSimple(String, NetworkTableType) */
    public boolean getBoolean(String name) { return getSimple(name, NetworkTableType.BOOLEAN).getBoolean(); }

    /** @see #getSimple(String, NetworkTableType) */
    public String getString(String name) { return getSimple(name, NetworkTableType.STRING).getString(); }

    /** @see #getSimple(String, NetworkTableType) */
    public long getInt(String name) { return getSimple(name, NetworkTableType.INTEGER).getInteger(); }

    /** @see #getSimple(String, NetworkTableType) */
    public float getLong(String name) { return getSimple(name, NetworkTableType.FLOAT).getFloat(); }

    /** @see #getSimple(String, NetworkTableType) */
    public byte[] getRaw(String name) { return getSimple(name, NetworkTableType.RAW).getRaw(); }

    /** @see #getSimple(String, NetworkTableType) */
    public double[] getDoubleArray(String name) {
        return getSimple(name, NetworkTableType.DOUBLE_ARRAY).getDoubleArray();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public boolean[] getBooleanArray(String name) {
        return getSimple(name, NetworkTableType.BOOLEAN_ARRAY).getBooleanArray();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public String[] getStringArray(String name) {
        return getSimple(name, NetworkTableType.STRING_ARRAY).getStringArray();
    }

    /** @see #getSimple(String, NetworkTableType) */
    public long[] getIntArray(String name) { return getSimple(name, NetworkTableType.INTEGER_ARRAY).getIntegerArray(); }

    /** @see #getSimple(String, NetworkTableType) */
    public float[] getFloatArray(String name) { return getSimple(name, NetworkTableType.FLOAT_ARRAY).getFloatArray(); }

    /**
     * Attempts to retrieve and unpack a struct under the given path in
     * NetworkTables.
     *
     * <p>
     * If the given entry name does not currently store a raw value, this function
     * will error out like {@link #getRaw}. Further, if the value can be retrieved
     * but not unpacked, a warning is reported to the DriverStation.
     *
     * <p>
     * Note that this function is thoroughly untested as of now.
     *
     * @param name   the name of the entry
     * @param struct the struct by whose rules to unpack the data
     *
     * @return the unpacked struct
     */
    public <T> T getStruct(String name, Struct<T> struct) {
        byte[] raw = getRaw(name);
        if (raw.length == 0) {
            return null;
        }
        try {
            StructBuffer<T> buffer = StructBuffer.create(struct);
            return buffer.read(raw);
        } catch (RuntimeException e) {
            DriverStationErrors.reportWarning(
                "NTable entry " + table.getPath() + "/" + name + " could not be unpacked: " + e.getMessage(), true);
            return null;
        }
    }

    /**
     * Attempts to retrieve and unpack an array of structs under the given path in
     * NetworkTables.
     *
     * <p>
     * If the given entry name does not currently store a raw value, this function
     * will error out like {@link #getRaw}. Further, if the value can be retrieved
     * but not unpacked, a warning is reported to the DriverStation.
     *
     * <p>
     * Note that this function is thoroughly untested as of now.
     *
     * @param name   the name of the entry
     * @param struct the struct by whose rules to unpack the data
     *
     * @return the unpacked struct
     */
    public <T> T[] getStructArray(String name, Struct<T> struct) {
        byte[] raw = getRaw(name);
        if (raw.length == 0) {
            return null;
        }
        try {
            StructBuffer<T> buffer = StructBuffer.create(struct);
            return buffer.readArray(raw);
        } catch (RuntimeException e) {
            DriverStationErrors.reportWarning(
                "NTable entry " + table.getPath() + "/" + name + " could not be unpacked: " + e.getMessage(), true);
            return null;
        }
    }

    public <T> Tunable<T> tunable(String name, Supplier<T> getter, Consumer<T> setCallback, Class<T> classType,
                                  TunableConfig config) {
        Tunable<T> res = Tunable.createConfig(null, null, classType, config);
        TunableRegistry.publish(getPath() + name, res);
        return res;
    }

    public <T> Tunable<T> tunable(String name, Supplier<T> getter, Consumer<T> setCallback, Class<T> classType) {
        return tunable(name, getter, setCallback, classType, null);
    }

    public <T> Tunable<T> tunable(String name, T initialValue, TunableConfig config) {
        Tunable<T> res = Tunable.createConfig(initialValue, config);
        TunableRegistry.publish(getPath() + name, res);
        return res;
    }

    public <T> Tunable<T> tunable(String name, T initialValue) { return tunable(name, initialValue, null); }

    public <T> Tunable<T> tunable(String name, Class<T> classType, TunableConfig config) {
        Tunable<T> res = Tunable.createNullConfig(classType, config);
        TunableRegistry.publish(getPath() + name, res);
        return res;
    }

    public <T> Tunable<T> tunable(String name, Class<T> classType) { return tunable(name, classType, null); }

    private <T extends TunableBase, U> T tunableOf(String name, Class<T> classType, U initialValue, Class<U> valueType) {
        try {
            Constructor<T> ctor = classType.getConstructor(valueType, TunableConfig.class);
            T res = ctor.newInstance(initialValue, TunableConfig.of(TunableOption.ALWAYS_GET));
            TunableRegistry.publish(getPath() + name, res);
            return res;
        } catch (ReflectiveOperationException e) {
            DriverStationErrors.reportWarning("some error occurred while making a tunable of type " + classType.getCanonicalName(), true);
            DriverStationErrors.reportWarning("error: " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public TunableInt tunableInt(String name, int initialValue) {
        return tunableOf(name, TunableInt.class, initialValue, int.class);
    }

    public TunableLong tunableLong(String name, long initialValue) {
        return tunableOf(name, TunableLong.class, initialValue, long.class);
    }

    public TunableFloat tunableFloat(String name, float initialValue) {
        return tunableOf(name, TunableFloat.class, initialValue, float.class);
    }

    public TunableDouble tunableDouble(String name, double initialValue) {
        return tunableOf(name, TunableDouble.class, initialValue, double.class);
    }

    public TunableBoolean tunableBoolean(String name, boolean initialValue) {
        return tunableOf(name, TunableBoolean.class, initialValue, boolean.class);
    }

    public TunableInt tunableInt(String name) {
        return tunableOf(name, TunableInt.class, 0, int.class);
    }

    public TunableLong tunableLong(String name) {
        return tunableOf(name, TunableLong.class, 0l, long.class);
    }

    public TunableFloat tunableFloat(String name) {
        return tunableOf(name, TunableFloat.class, 0f, float.class);
    }

    public TunableDouble tunableDouble(String name) {
        return tunableOf(name, TunableDouble.class, 0d, double.class);
    }

    public TunableBoolean tunableBoolean(String name) {
        return tunableOf(name, TunableBoolean.class, false, boolean.class);
    }

    // listen for changes to an entry
    public <T> void listen(String name, Consumer<T> consumer, Class<T> classType) {
        tunable(name, null, consumer, classType, TunableConfig.of(TunableOption.GET_ON_CHANGE));
    }

    /** {@return whether the given name is present in this NTable with that type} */
    public boolean existsAs(String name, NetworkTableType desiredType) {
        return getEntry(name).getType().equals(desiredType);
    }

    /** {@return whether the given name is present on NetworkTables} */
    public boolean exists(String name) { return getEntry(name).exists(); }

    /** {@return whether all of the given names are present in this NTable} */
    public boolean exists(String... names) { return Arrays.stream(names).allMatch(name -> exists(name)); }

    /**
     * {@return whether the given name is present in this NTable, and if not, sets
     * it to the passed value}
     */
    public <T> boolean ensure(String name, T value) {
        if (!exists(name)) {
            set(name, value);
            makePersistent(name);
            return true;
        }
        return false;
    }

    /**
     * Makes the specified entries persist through program restarts.
     *
     * @param names the names of the entries
     */
    public void makePersistent(String... names) {
        for (String name : names) {
            table.getEntry(name).setPersistent();
        }
    }

    /**
     * Makes the specified entries not persist through program restarts.
     *
     * @param names the names of the entries
     */
    public void clearPersistent(String... names) {
        for (String name : names) {
            table.getEntry(name).clearPersistent();
        }
    }
}
