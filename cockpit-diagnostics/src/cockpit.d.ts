// SPDX-License-Identifier: LGPL-2.1-or-later
// The permission API implemented by the pinned Cockpit runtime, not yet present in its copied declarations.

import "cockpit";

declare module "cockpit" {
    interface Permission extends EventTarget {
        allowed: boolean | null;
        close(): void;
    }

    function permission(options: { admin: boolean }): Permission;
}
