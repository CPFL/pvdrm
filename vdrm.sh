#!/bin/bash
# usage: ./vdrm.sh vdrm 1
if [ $# != 2 ]
then
        echo "Usage: $0 <device name> <frontend-id>"
else
        # Write backend information into the location the frontend will look
        # for it.
        xenstore-write /local/domain/${2}/device/${1}/0/backend-id 0
        xenstore-write /local/domain/${2}/device/${1}/0/error 0
        xenstore-write /local/domain/${2}/device/${1}/0/backend /local/domain/0/backend/${1}/${2}/0
        # Write frontend information into the location the backend will look
        # for it.
        xenstore-write /local/domain/0/backend/${1}/${2}/0/frontend-id ${2}
        xenstore-write /local/domain/0/backend/${1}/${2}/0/frontend /local/domain/${2}/device/${1}/0
        # Set the permissions on the backend so that the frontend can
        # actually read it.
        xenstore-chmod /local/domain/0/backend/${1}/${2}/0 r
        xenstore-chmod /local/domain/${2}/device/${1}/0 b
        # xenstore-chmod /local/domain/${2}/error/device/${1}/0 b
        # Write the states.  Note that the backend state must be written
        # last because it requires a valid frontend state to already be
        # written.
        xenstore-write /local/domain/${2}/device/${1}/0/state 1
        xenstore-write /local/domain/0/backend/${1}/${2}/0/state 1
fi
