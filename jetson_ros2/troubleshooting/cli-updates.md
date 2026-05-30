## Troubleshooting CLI Updates and Command Path Cache

When installing or updating a command-line tool, the installation may complete successfully, but the terminal may still execute an older version of the command.

This usually happens because shells such as `bash` and `zsh` can cache command locations. The shell remembers where an executable was found in the `$PATH` to avoid searching for it every time. After an update, reinstall, or path change, the cached location may still point to an older binary.

As a result, a command may continue showing the previous version even though the package manager reports that the new version is already installed.

---

## Common Symptoms

You may be facing a shell path cache or binary resolution problem if one of these cases happens:

```bash
tool --version
```

still shows the old version after updating.

```bash
package-manager list --global
```

shows the new version installed, but the command still runs the old one.

```bash
which tool
```

points to a different location than expected.

```bash
type -a tool
```

shows multiple binaries with the same command name.

Example:

```bash
tool is /home/user/.nvm/versions/node/v18.x.x/bin/tool
tool is /usr/local/bin/tool
tool is /usr/bin/tool
```

In this case, the terminal may be executing the first matching binary found in `$PATH`, or it may still be using a cached command path.

---

## Step 1: Check the Installed Version

First, confirm that the tool was actually updated by checking it through the package manager.

For npm global packages:

```bash
npm list -g --depth=0
```

For Python packages installed with pip:

```bash
pip show package-name
```

or:

```bash
python -m pip show package-name
```

For system packages:

```bash
apt policy package-name
```

or:

```bash
dpkg -l | grep package-name
```

The goal is to verify whether the package manager sees the new version.

---

## Step 2: Check Which Binary Is Being Executed

Use:

```bash
which tool
```

Example:

```bash
which node
which npm
which python
which tool-name
```

This shows the executable path that the shell is currently resolving.

Then check all matching binaries:

```bash
type -a tool
```

Example:

```bash
type -a node
type -a npm
type -a python
type -a tool-name
```

If multiple paths appear, the system has more than one executable with the same command name.

Example:

```bash
tool is /home/user/.local/bin/tool
tool is /usr/local/bin/tool
tool is /usr/bin/tool
```

The order matters. The first result is usually the one that will be executed.

---

## Step 3: Clear the Shell Command Cache

After installing or updating a CLI tool, refresh the shell command cache:

```bash
hash -r
```

Then check the version again:

```bash
tool --version
```

In `zsh`, you can also use:

```bash
rehash
```

This forces the shell to forget cached command paths and search the `$PATH` again.

This is especially useful after:

* Updating a global CLI tool
* Installing a new binary
* Removing an old binary
* Changing `$PATH`
* Switching Node.js versions with a version manager
* Switching Python environments
* Moving a command from one directory to another
* Replacing a system-wide install with a user-level install

---

## Step 4: Check the `$PATH` Order

If the command still runs the wrong version, inspect the `$PATH`:

```bash
echo $PATH
```

For easier reading:

```bash
echo $PATH | tr ':' '\n'
```

The shell searches these directories from top to bottom. If an old binary appears in a directory listed before the new binary, the old one may be executed first.

Example problem:

```bash
/usr/local/bin
/home/user/.local/bin
/home/user/.nvm/versions/node/v18.x.x/bin
/usr/bin
```

If the updated binary is in:

```bash
/home/user/.nvm/versions/node/v18.x.x/bin
```

but an older binary exists in:

```bash
/usr/local/bin
```

then the older one may be used first.

---

## Step 5: Temporarily Test the Correct Binary

Run the expected binary directly using its full path:

```bash
/full/path/to/tool --version
```

Example:

```bash
/home/user/.local/bin/tool --version
```

If the full path shows the correct version, but the normal command does not, the issue is not the installation. The issue is command resolution through `$PATH`.

---

## Step 6: Fix the `$PATH` Order

If the correct binary is installed in a user-level directory, make sure that directory appears before system directories in `$PATH`.

For `zsh`, edit:

```bash
~/.zshrc
```

For `bash`, edit:

```bash
~/.bashrc
```

Add the correct directory near the top:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

For Node.js installed with `nvm`, make sure the `nvm` initialization block is present in your shell config file.

After editing the shell configuration, reload it:

```bash
source ~/.zshrc
```

or:

```bash
source ~/.bashrc
```

Then clear the command cache:

```bash
hash -r
```

Finally, verify:

```bash
which tool
tool --version
```

---

## Step 7: Rename or Remove Old Binaries Carefully

If an old binary is still being found first, identify it with:

```bash
type -a tool
```

If you are sure it is obsolete, rename it instead of deleting it immediately:

```bash
sudo mv /usr/local/bin/tool /usr/local/bin/tool.old
```

Then refresh the shell cache:

```bash
hash -r
```

Check again:

```bash
which tool
tool --version
```

Renaming is safer than deleting because it allows you to restore the old binary if needed.

---

## Recommended Debugging Checklist

Use this sequence when a CLI tool does not update correctly:

```bash
tool --version
which tool
type -a tool
echo $PATH | tr ':' '\n'
hash -r
tool --version
```

If the version is still wrong:

```bash
type -a tool
/full/path/to/expected/tool --version
```

If the full path works, fix the `$PATH` order or remove the old binary from the earlier path.

---

## Common Fix Pattern

A common fix after updating a CLI tool is:

```bash
hash -r
which tool
tool --version
```

If using `zsh`, this can also be used:

```bash
rehash
which tool
tool --version
```

If the wrong binary is still being used:

```bash
type -a tool
echo $PATH | tr ':' '\n'
```

Then update the `$PATH` or rename the obsolete binary.

---

## Summary

When a CLI tool update appears successful but the terminal still runs the old version, the most common causes are:

1. The shell is using a cached command path.
2. Multiple versions of the same command exist.
3. `$PATH` prioritizes an older binary.
4. The package manager updated one installation, but the shell is executing another.

The first fix to try is:

```bash
hash -r
```

or, in `zsh`:

```bash
rehash
```

Then verify the result with:

```bash
which tool
type -a tool
tool --version
```

If the problem continues, inspect the `$PATH` order and remove or rename obsolete binaries carefully.
