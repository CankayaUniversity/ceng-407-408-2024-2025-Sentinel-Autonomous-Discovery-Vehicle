import { ItemActionTypes } from '../../definitions/actionTypeDefinitions';

interface ItemState {
    items: { id: number; name: string }[];
}

const initialState: ItemState = {
    items: [],
};

const itemReducer = (state: ItemState = initialState, action: any): ItemState => {
    switch (action.type) {
        case ItemActionTypes.ADD_ITEM:
            return { ...state, items: [...state.items, action.payload] };

        case ItemActionTypes.UPDATE_ITEM:
            return {
                ...state,
                items: state.items.map((item) =>
                    item.id === action.payload.id ? action.payload : item
                ),
            };

        case ItemActionTypes.DELETE_ITEM:
            return {
                ...state,
                items: state.items.filter((item) => item.id !== action.payload.id),
            };

        default:
            return state;
    }
};

export default itemReducer;