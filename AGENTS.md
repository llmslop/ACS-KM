AGENTS
======

This repository contains a Java implementation of an ACO-based solver for (D)VRPTW.
This document is written specifically for agentic coding agents (and humans) that
work on the codebase. It describes how to build/run the project, how to run a
single experiment, and the code-style and operational rules agents must follow.

Keep this file concise, actionable and machine-friendly. Refer to specific files
by path when giving instructions.

---

1) Quick build & run
---------------------

- Build all sources (simple, no build system required):

  ```sh
  # from repository root
  javac -d bin -cp "lib/*" $(find src -name "*.java")
  ```

- Run the main controller (example):

  ```sh
  java -cp "bin:lib/*" aco.Controller [args]
  ```

- Nix development (reproducible):

  ```sh
  nix develop --command bash -lc 'javac -d bin -cp "lib/*" $(find src -name "*.java") && java -cp "bin:lib/*" aco.Controller'
  ```

- Example: run a single ACS run and set objective weights (cost and rejects):

  ```sh
  java -cp "bin:lib/*" aco.Controller --acs-km --cost-weight 1.0 --reject-weight 10.0
  ```

- Time limits: you can now control solver time from the CLI:

  ```sh
  java -cp "bin:lib/*" aco.Controller --acs-km --cost-weight 1.0 --reject-weight 10.0 --time-limit 5.0 --working-day 60
  ```

  Flags added: `-T/--time-limit` sets `InOut.max_time` (seconds, double, >=0) and
  `-W/--working-day` sets the simulated `Controller.workingDay` (seconds, double, >=0).

  Notes: The CLI parsing is in `src/aco/Parse.java`. Flags added by agents: `-C/--cost-weight` and
  `-R/--reject-weight` (short: `-C`, `-R`) set `InOut.costWeight` and `InOut.rejectWeight`.

2) Tests / single-test run
--------------------------

- This repository does not include a unit-test framework by default. Use these
  patterns when adding tests:

  - Add JUnit tests under `test/` and include a simple runner script.
  - To run a single test class (once tests are added via a classpath):

    ```sh
    # compile tests with the same classpath as main
    javac -d bin -cp "lib/*:bin" $(find test -name "*.java")
    # run a single JUnit test via the runner you choose (or use a small main wrapper)
    ```

- For quick sanity checks run a single main entry point that exercises key logic,
  e.g. `aco.VRPTW_ACS` has a `main` method for running the solver on an input file.

3) Linting & formatting
------------------------

- There is no project linter configured. Recommended setup for consistent style:

  - Use Google Java Format for automated formatting. Example usage (download jar):

    ```sh
    java -jar google-java-format-1.15.0-all-deps.jar --replace $(find src -name "*.java")
    ```

  - Run `javac -Xlint:all` to surface suspicious code (unused imports,
    unchecked casts, deprecations). Fix warnings before committing.

- Import ordering and style (enforced informally):

  1. `java.*`, `javax.*`
  2. `org.*` (third-party)
  3. `aco.*` (local packages)

  Always use explicit imports (no wildcard imports) for readability.

4) Code style guidelines (Java-specific)
----------------------------------------

- Encoding and characters
  - Default to ASCII in new files. Only add non-ASCII if a file already uses them
    and there's a clear reason (user-visible text/strings). This keeps tools
    portable.

- Formatting
  - 4-space indentation.
  - Use Google Java Format (recommended) for line-wrapping, brace positions, etc.

- Naming conventions
  - Packages: `lowercase` (existing `aco` package is correct).
  - Classes: `UpperCamelCase` (e.g., `DataReader`, `VRPTW_ACS`).
  - Methods/variables: `camelCase`.
  - Constants: `UPPER_SNAKE_CASE` (static final), use `private` visibility unless needed.

- Types and API
  - Prefer primitive types for numeric counters (int, double) where overflow is not expected.
  - Use arrays for fixed-size numeric buffers (`double[] beginService` is fine) and
    ArrayList for dynamic collections. Keep consistent with existing patterns.

- Imports and package structure
  - Group imports by standard/third-party/local as above.
  - Do not add new top-level packages without a short justification.

- Comments
  - Keep comments short and use them to clarify non-obvious algorithms.
  - Avoid noisy comments that restate code. Prefer small Javadoc for public classes and methods.

- Error handling
  - Prefer specific exceptions. Do not swallow exceptions silently.
  - Use try-with-resources for IO (BufferedReader/Writer, streams).
  - If a method cannot recover from an error, either throw a checked exception or
    log a clear error message and exit cleanly from the top-level `main`.

- Logging
  - This project uses `System.out.println` in many places. New code may introduce
    a lightweight logger (e.g., java.util.logging) if needed — but be consistent.

5) Design & invariants
-----------------------

- Indexing conventions
  - Input file IDs: depot is `0`, customers start at `1`. Many internal arrays are
    zero-based for customers; some code uses `city + 1` when indexing `reqList` or
    `nodes[]`. Read `DataReader.read()` and `Request` carefully before changing code.

- Mutability
  - `Ant` objects are mutable and copied frequently using `Ants.copy_from_to`. When
    editing these utilities keep field-by-field copying consistent (rejected fields,
    visited flags, and lists).

- Objective change rules
  - The new weighted objective uses `InOut.costWeight` and `InOut.rejectWeight`.
    If you change objective logic, update all selection/pheromone/deposit code paths
    to remain consistent (find_best, pheromone updates, update_statistics, reporting).

6) Operational & git rules for agents
-------------------------------------

- NEVER execute destructive git commands without explicit user approval. Specifically
  avoid `git reset --hard`, `git checkout -- .`, or force pushes to protected branches.

- Do not revert unrelated changes in the working tree. If the workspace is dirty,
  only modify the files you were asked to change.

- Use `apply_patch` for programmatic edits (preferred) and `Read` / `Edit` / `Write`
  helpers for local file operations.

- Commits
  - Do not create commits unless the human user asks you to. If asked to commit,
    craft a concise commit message that explains the why, not just the what.

7) Agent behaviour & workflow
-----------------------------

- Default: make minimal, low-risk edits and avoid sweeping refactors.

- If you are blocked by ambiguous requirements, ask a single targeted question.

- When adding flags or CLI options, update `src/aco/Parse.java` and document the
  flag in this file or in `README.md`.

- Use tests or short runs to validate changes. If you cannot run the full test
  harness, run a small smoke test (compile + single example as shown above).

8) Cursor/Copilot rules
-----------------------

- Repo search: there are no Cursor rules in `.cursor/rules/` or `.cursorrules`.
  There is no `.github/copilot-instructions.md` either. If you need to adhere to
  additional organization-specific AI guidelines, consult the repository owner.

9) Where to look first when changing parsing or request logic
------------------------------------------------------------

- `src/aco/DataReader.java` — input parsing (NUMBER and CUST NO. sections).
- `src/aco/Request.java` — request data model (id, x,y, demand, windows, service, availableTime).
- `src/aco/VRPTW.java` — distance matrix and NN list helpers.
- `src/aco/Ants.java` and `src/aco/VRPTW_ACS.java` — core solver, pheromone updates
  and best-solutions bookkeeping.

10) Small checklist for reviews
------------------------------

- Does the change preserve indexing conventions (city vs. city+1)?
- Are rejected counts/lists handled when ants are copied or reset?
- If objective logic changes, are pheromone and best-so-far updates consistent?
- Are CLI flags validated (non-negative weights) and printed at startup?

---

If you have questions about where to run a command or need a short smoke-run, ask
and include the preferred inputs (which instance file under `input/` to use and any
desired flags like `--cost-weight`).
