## Personal ZMK Fork

Starts with this [base](https://github.com/paulshir/zmk/tree/pr2766%2Bpio-led) as base. This includes
* [Feature: Full-Duplex Wired Split](https://github.com/zmkfirmware/zmk/pull/20766)
* Zephyr branch with support for WS2812 on RP2040

Adds the following PRs:
* [Tri State](https://github.com/zmkfirmware/zmk/pull/1366)

### Resetting this branch

```
git reset --hard paulshir/main+pio-led
git cherry-pick readme
git merge tristate

git fetch upstream pull/1366/head:pr1366 --force

git merge pr1366
```

### To update this file
```
git checkout readme
vim README.md
git add --update
git commit --fixup HEAD
git rebase -i --autosquash HEAD~2
```

### To update the tristate branch (or fix merge conflicts)
```
git checkout tristate
git reset --hard upstream/main
git fetch upstream pull/1366/head:pr1366 --force
git merge pr1366
```

