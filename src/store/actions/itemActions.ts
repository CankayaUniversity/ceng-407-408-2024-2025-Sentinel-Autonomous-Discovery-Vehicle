import {
    AddItemAction,
    DeleteItemAction,
    Item,
    ItemActionTypes,
    UpdateItemAction
} from '../../definitions/actionTypeDefinitions';

export const addItem = (item: Item): AddItemAction => ({
    type: ItemActionTypes.ADD_ITEM,
    payload: item,
});

export const updateItem = (item: Item): UpdateItemAction => ({
    type: ItemActionTypes.UPDATE_ITEM,
    payload: item,
});

export const deleteItem = (id: number): DeleteItemAction => ({
    type: ItemActionTypes.DELETE_ITEM,
    payload: { id },
});
