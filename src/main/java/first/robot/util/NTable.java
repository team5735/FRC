package first.robot.util;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableType;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.telemetry.TelemetryTable;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
    // All fields and static fields are package-private for NTEntry.
    static final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    static final NTable root = new NTable(instance.getTable(""), null);

    final NetworkTable table;
    final NTable parent;
    final TelemetryTable telemetryTable;

    Map<String, NTEntry> entries = new HashMap<>();

    /** {@return the parent of this NTable, or null if this == NTable.root()} */
    public NTable getParent() { return this.parent; }

    /** {@return the underlying {@link NetworkTable}} */
    public NetworkTable getTable() { return this.table; }

    /** {@return the path of this NTable} */
    public String getPath() { return this.table.getPath(); }

    NTable(NetworkTable table, NTable parent) {
        this.table = table;
        this.parent = parent;
        this.telemetryTable = TelemetryRegistry.getTable(getPath());

        long depth = this.table.getPath().chars().filter(c -> c == '/').count();
        if (depth > 50) {
            DriverStationErrors.reportWarning(
                "very long NTable of depth " + depth + " created! be careful. its path is " + this.table.getPath(), true
            );
        }

        for (String name : table.getKeys()) {
            entries.put(name, new NTEntry(this, name, table.getEntry(name)));
        }
    }

    /** {@return the root NTable} */
    public static NTable root() { return root; }

    /** {@return NTable.root().sub(name)} */
    public static NTable root(String name) { return root().sub(name); }

    /** {@return a subtable of this NTable} */
    public NTable sub(String name) { return this.entry(name).getAsNTable(); }

    /**
     * Given a TelemetryTable, construct and return an NTable representing the same
     * NetworkTable. This creates all intermediate NTables in the cache.
     *
     * @param table the TelemetryTable to copy from
     *
     * @return NTable such that NTable.telemetryTable() == table
     */
    public static NTable fromTelemetryTable(TelemetryTable table) {
        String path[] = table.getPath().split("/");
        List<String> stream = Arrays.stream(path).skip(1).limit(path.length - 1).toList();
        NTable working = NTable.root();
        for (String pathComponent : stream) {
            working = working.sub(pathComponent);
        }
        assert working.telemetryTable() == table;
        return working;
    }

    /** {@return an entry of this NTable} */
    public NTEntry entry(String name) { return this.entries.computeIfAbsent(name, n -> new NTEntry(this, n)); }

    public String pathOf(String entryName) { return this.getPath() + "/" + entryName; }

    /** Equivalent to {@code this.entry(name).set(value)} */
    public <T> void set(String name, T value) { entry(name).set(value); }

    /** Equivalent to {@code this.entry(name).get(defaultValue)} */
    public <T> T get(String name, T defaultValue) { return entry(name).get(defaultValue); }

    /** {@return the {@link TelemetryTable} for this NTable} */
    public TelemetryTable telemetryTable() { return this.telemetryTable; }

    /** {@return whether the given name is present in this NTable with that type} */
    public boolean existsAs(String name, NetworkTableType desiredType) {
        return entry(name).getType().equals(desiredType);
    }

    /** {@return whether the given name is present on NetworkTables} */
    public boolean exists(String name) { return entry(name).exists(); }

    /** {@return whether all of the given names are present in this NTable} */
    public boolean exists(String... names) { return Arrays.stream(names).allMatch(name -> exists(name)); }
}
