export interface Item {
    id: number;
    name: string;
}

export interface AddItemAction {
    type: ItemActionTypes.ADD_ITEM;
    payload: Item;
}

export interface UpdateItemAction {
    type: ItemActionTypes.UPDATE_ITEM;
    payload: Item;
}

export interface DeleteItemAction {
    type: ItemActionTypes.DELETE_ITEM;
    payload: { id: number };
}

export enum ItemActionTypes {
    ADD_ITEM = 'ADD_ITEM',
    UPDATE_ITEM = 'UPDATE_ITEM',
    DELETE_ITEM = 'DELETE_ITEM',
}

export type ItemActions = AddItemAction | UpdateItemAction | DeleteItemAction;
